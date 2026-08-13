#!/usr/bin/env bash
# build.sh — 构建硬件驱动镜像并推送到镜像仓库
#
# 用法：bash build.sh [--mirror tuna|tencent|none] [--no-cache] [driver_dir...]
#   不传参数时显示交互式多选框
#   直接传目录名时跳过选择（CI 用）
#
# 依赖：
#   - 每个驱动目录下需有 driver.yaml 和 Dockerfile
#   - python3（解析 YAML）或 yq（可选）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${SCRIPT_DIR}/.env"

# ── 加载 .env ──────────────────────────────────────────────────────────────
if [ -f "${ENV_FILE}" ]; then
    source "${ENV_FILE}"
fi

# ── 解析参数 ──────────────────────────────────────────────────────────────
MIRROR="${MIRROR:-}"
NO_CACHE=""
REMAINING_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mirror) MIRROR="$2"; shift 2 ;;
        --no-cache) NO_CACHE="--no-cache"; shift ;;
        *) REMAINING_ARGS+=("$1"); shift ;;
    esac
done
set -- "${REMAINING_ARGS[@]+"${REMAINING_ARGS[@]}"}"

# ── 镜像源选择 ────────────────────────────────────────────────────────────
select_mirror() {
    if [ -z "${MIRROR}" ]; then
        choice=""
        if [ -t 0 ]; then
            echo ""
            echo "Select mirror / 选择镜像源:"
            echo "  1) tencent  — 腾讯云（VPC 内网）"
            echo "  2) tuna     — 清华 TUNA（公网）"
            echo "  3) none     — 官方源（海外 / 裸连）"
            printf "Choice [1/2/3] (default: 2): "
            read -r choice || choice=""
        fi
        case "${choice}" in
            1) MIRROR="tencent" ;;
            3) MIRROR="none" ;;
            *) MIRROR="tuna" ;;
        esac
    fi

    case "${MIRROR}" in
        tencent)
            PYPI_MIRROR="https://mirrors.tencentyun.com/pypi/simple/"
            ROS_APT_MIRROR="${ROS_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
            UBUNTU_APT_MIRROR="${UBUNTU_APT_MIRROR:-http://mirrors.tencentyun.com/ubuntu-ports}"
            BINFMT_IMAGE="mirror.ccs.tencentyun.com/tonistiigi/binfmt"
            ;;
        tuna)
            PYPI_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple/"
            ROS_APT_MIRROR="${ROS_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
            UBUNTU_APT_MIRROR="${UBUNTU_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports}"
            BINFMT_IMAGE="docker.io/tonistiigi/binfmt"
            ;;
        none|*)
            PYPI_MIRROR="https://pypi.org/simple/"
            ROS_APT_MIRROR="${ROS_APT_MIRROR:-http://packages.ros.org/ros2/ubuntu}"
            UBUNTU_APT_MIRROR="${UBUNTU_APT_MIRROR:-http://ports.ubuntu.com/ubuntu-ports}"
            BINFMT_IMAGE="docker.io/tonistiigi/binfmt"
            ;;
    esac

    echo "Mirror: ${MIRROR} | PyPI: ${PYPI_MIRROR} | ROS apt: ${ROS_APT_MIRROR} | Ubuntu apt: ${UBUNTU_APT_MIRROR}"
    echo ""
}

# If registry not configured, build locally only
PUSH_ENABLED=true
if [ -z "${REGISTRY:-}" ] || [ -z "${REGISTRY_USER:-}" ] || [ -z "${REGISTRY_PASSWORD:-}" ] || [ -z "${IMAGE_NAMESPACE:-}" ]; then
    echo "[info] Registry not configured — building locally only (no push)."
    PUSH_ENABLED=false
    REGISTRY="${REGISTRY:-local}"
    IMAGE_NAMESPACE="${IMAGE_NAMESPACE:-phanthy-motus/drivers}"
fi

RESOURCE_CENTER_URL="${RESOURCE_CENTER_URL:-https://motus.phanthy.com}"

# ── 发现可构建的驱动 ────────────────────────────────────────────────────────
declare -a DRIVER_DIRS
declare -a DRIVER_NAMES
declare -a DRIVER_IMAGES
declare -a DRIVER_IDS
declare -a DRIVER_PORTS
declare -a DRIVER_MCPS
declare -a DRIVER_DESCS
declare -a DRIVER_CATS
declare -a DRIVER_PROVIDERS
declare -a DRIVER_MODELS
declare -a DRIVER_BUILDABLE   # "yes" / "no (no Dockerfile)"

_parse_yaml_field() {
    local file="$1" field="$2" val
    val=$(grep -m1 "^${field}:" "${file}" | cut -d: -f2- | xargs 2>/dev/null) || true
    val="${val#[\"\']}"
    val="${val%[\"\']}"
    echo "${val}"
}

_parse_yaml_list() {
    # Extract items under a YAML list key (simple single-level list)
    local file="$1" field="$2"
    awk "/^${field}:/{found=1; next} found && /^  - /{print \$2; next} found && !/^  /{exit}" "${file}" 2>/dev/null || true
}

for yaml_file in "${SCRIPT_DIR}"/*/*/driver.yaml "${SCRIPT_DIR}"/*/driver.yaml; do
    [ -f "${yaml_file}" ] || continue
    dir="$(dirname "${yaml_file}")/"
    has_dockerfile="yes"
    [ -f "${dir}Dockerfile" ] || has_dockerfile="no (no Dockerfile)"

    DRIVER_DIRS+=("${dir}")
    DRIVER_NAMES+=("$(_parse_yaml_field "${yaml_file}" name)")
    DRIVER_IMAGES+=("$(_parse_yaml_field "${yaml_file}" image_name)")
    DRIVER_IDS+=("$(_parse_yaml_field "${yaml_file}" id)")
    DRIVER_PORTS+=("$(_parse_yaml_field "${yaml_file}" port)")
    DRIVER_MCPS+=("$(_parse_yaml_field "${yaml_file}" mcp_url)")
    DRIVER_DESCS+=("$(_parse_yaml_field "${yaml_file}" description)")
    DRIVER_CATS+=("$(_parse_yaml_field "${yaml_file}" category)")
    DRIVER_PROVIDERS+=("$(_parse_yaml_field "${yaml_file}" hardware_provider)")
    DRIVER_MODELS+=("$(_parse_yaml_field "${yaml_file}" hardware_model)")
    DRIVER_BUILDABLE+=("${has_dockerfile}")
done

if [ ${#DRIVER_DIRS[@]} -eq 0 ]; then
    echo "错误：未找到任何 driver.yaml 文件"
    exit 1
fi

# ── 选择要构建的驱动 ────────────────────────────────────────────────────────
declare -a SELECTED_INDICES

if [ $# -gt 0 ]; then
    # CI may identify a driver by directory, manifest id, image name, model,
    # or provider/model. Match exact aliases so a typo cannot silently no-op.
    for arg in "$@"; do
        normalized_arg="${arg#./}"
        normalized_arg="${normalized_arg%/}"
        matched=false
        for i in "${!DRIVER_DIRS[@]}"; do
            relative_dir="${DRIVER_DIRS[$i]#${SCRIPT_DIR}/}"
            relative_dir="${relative_dir%/}"
            absolute_dir="${DRIVER_DIRS[$i]%/}"
            provider_model="${DRIVER_PROVIDERS[$i]}/${DRIVER_MODELS[$i]}"
            case "${normalized_arg}" in
                "${relative_dir}"|"${absolute_dir}"|"${DRIVER_IDS[$i]}"|"${DRIVER_IMAGES[$i]}"|\
                "${DRIVER_MODELS[$i]}"|"${provider_model}")
                    if [[ " ${SELECTED_INDICES[*]-} " != *" ${i} "* ]]; then
                        SELECTED_INDICES+=("$i")
                    fi
                    matched=true
                    ;;
            esac
            # Existing preview deployments used t800-dev before the manifest
            # was normalized to t800. Accept those identifiers as migration
            # aliases, while always building and registering the canonical
            # engineai/t800 image.
            if [ "${relative_dir}" = "engineai/t800" ]; then
                case "${normalized_arg}" in
                    t800-dev|engineai/t800-dev|engineai-t800-dev)
                        if [[ " ${SELECTED_INDICES[*]-} " != *" ${i} "* ]]; then
                            SELECTED_INDICES+=("$i")
                        fi
                        echo "[warn] legacy T800 alias '${arg}' maps to canonical engineai/t800." >&2
                        matched=true
                        ;;
                esac
            fi
        done
        if ! ${matched}; then
            echo "错误：未知驱动参数 '${arg}'。请使用目录、driver id、image_name、hardware_model 或 provider/model。" >&2
            exit 2
        fi
    done
else
    if [ ! -t 0 ]; then
        echo "错误：非交互环境必须显式传入 driver 参数。" >&2
        exit 2
    fi
    # 交互式选择（InquirerPy）——写入临时文件以便从 /dev/tty 读取键盘输入
    _PY_SEL=$(mktemp /tmp/build_select_XXXXXX.py)
    cat > "${_PY_SEL}" <<'PYEOF'
import sys, os
try:
    from InquirerPy import inquirer
except ImportError:
    sys.stderr.write("请先安装 InquirerPy: pip install InquirerPy\n")
    sys.exit(1)

total       = int(sys.argv[1])
names       = sys.argv[2:2+total]
buildables  = sys.argv[2+total:2+2*total]
providers   = sys.argv[2+2*total:2+3*total]
models      = sys.argv[2+3*total:2+4*total]

choices = []
for i in range(total):
    label = f"{providers[i]}/{models[i]}  ({names[i]})"
    enabled = buildables[i] == "yes"
    if not enabled:
        label += f"  [{buildables[i]}]"
    choices.append({"name": label, "value": str(i), "enabled": False})

# 把 stdout 重定向到 /dev/tty，让 UI 渲染到终端而不被 $() 吞掉
real_stdout_fd = os.dup(1)
tty = open("/dev/tty", "w")
os.dup2(tty.fileno(), 1)

results = inquirer.checkbox(
    message="选择要构建的驱动（空格选中，回车确认，a 全选）：",
    choices=choices,
).execute()

# 恢复真实 stdout，只输出结果
os.dup2(real_stdout_fd, 1)
os.close(real_stdout_fd)
tty.close()

print(" ".join(results))
PYEOF
    SELECTED_INDICES_STR=$(python3 "${_PY_SEL}" \
        "${#DRIVER_DIRS[@]}" \
        "${DRIVER_NAMES[@]}" \
        "${DRIVER_BUILDABLE[@]}" \
        "${DRIVER_PROVIDERS[@]}" \
        "${DRIVER_MODELS[@]}" </dev/tty)
    rm -f "${_PY_SEL}"

    if [ -z "${SELECTED_INDICES_STR}" ]; then
        echo "未选择任何驱动，退出。"
        exit 0
    fi

    for idx in ${SELECTED_INDICES_STR}; do
        SELECTED_INDICES+=("${idx}")
    done
fi

if [ ${#SELECTED_INDICES[@]} -eq 0 ]; then
    echo "错误：未选择任何驱动。" >&2
    exit 2
fi

# ── 检查选中 driver 是否可构建 ────────────────────────────────────────────
for idx in "${SELECTED_INDICES[@]}"; do
    buildable="${DRIVER_BUILDABLE[$idx]}"
    if [ "${buildable}" != "yes" ]; then
        echo "错误：${DRIVER_NAMES[$idx]} ${buildable}"
        exit 1
    fi
done

# ── 生成版本号 ─────────────────────────────────────────────────────────────
DATE="$(date +%y%m%d)"
COMMIT="$(git -C "${SCRIPT_DIR}" rev-parse --short=7 HEAD 2>/dev/null || echo "local")"
DIRTY_SUFFIX=""
if [ "${COMMIT}" = "local" ] && ${PUSH_ENABLED}; then
    echo "错误：发布镜像需要可读取的 Git commit，当前工作副本没有可用版本元数据。" >&2
    exit 1
fi
if [ -n "$(git -C "${SCRIPT_DIR}" status --porcelain --untracked-files=normal 2>/dev/null || true)" ]; then
    if ${PUSH_ENABLED}; then
        echo "错误：工作树包含未提交改动，拒绝发布无法复现的镜像。" >&2
        exit 1
    fi
    DIRTY_SUFFIX=".dirty"
fi
TAG="release.${DATE}.${COMMIT}${DIRTY_SUFFIX}"

echo ""
echo "版本 tag：${TAG}"
echo "目标仓库：${REGISTRY}/${IMAGE_NAMESPACE}/"
echo ""

# ── 登录 & QEMU ───────────────────────────────────────────────────────────
if ${PUSH_ENABLED}; then
    echo "${REGISTRY_PASSWORD}" | docker login "${REGISTRY}" -u "${REGISTRY_USER}" --password-stdin
fi

select_mirror

# ARM64 hosts (including Apple Silicon Docker Desktop) build linux/arm64
# natively. Registering binfmt there is unnecessary and may fail because
# Docker Desktop does not expose /proc/sys/fs/binfmt_misc/register.
HOST_ARCH="$(uname -m)"
# Some rootless Docker setups print a successful binfmt installation even
# though the daemon cannot execute ARM64 layers. Probe execution before a
# large build so the failure is immediate and actionable.
BINFMT_PROBE_IMAGE="${BINFMT_PROBE_IMAGE:-docker.io/library/alpine:3.20}"
PROBE_ARCH="$(docker run --rm --platform linux/arm64 "${BINFMT_PROBE_IMAGE}" uname -m 2>/dev/null || true)"
if [ "${HOST_ARCH}" != "arm64" ] && [ "${HOST_ARCH}" != "aarch64" ] && \
   [[ "${PROBE_ARCH}" != "arm64" && "${PROBE_ARCH}" != "aarch64" ]]; then
    docker run --privileged --rm "${BINFMT_IMAGE}" --install arm64
    PROBE_ARCH="$(docker run --rm --platform linux/arm64 "${BINFMT_PROBE_IMAGE}" uname -m 2>/dev/null || true)"
elif [ "${HOST_ARCH}" = "arm64" ] || [ "${HOST_ARCH}" = "aarch64" ]; then
    echo "[info] Native ${HOST_ARCH} host — skipping ARM64 binfmt setup."
else
    echo "[info] ARM64 emulation already available — skipping binfmt registration."
fi
if [[ "${PROBE_ARCH}" != "arm64" && "${PROBE_ARCH}" != "aarch64" ]]; then
    echo "错误：当前 Docker daemon 无法执行 linux/arm64 容器。请在宿主机启用 QEMU binfmt，并重启 Docker daemon。" >&2
    exit 1
fi
echo "[info] ARM64 execution probe passed (${PROBE_ARCH})."

# ── 构建 ──────────────────────────────────────────────────────────────────
declare -a BUILT_INDICES

for idx in "${SELECTED_INDICES[@]}"; do
    dir="${DRIVER_DIRS[$idx]}"
    name="${DRIVER_NAMES[$idx]}"
    provider="${DRIVER_PROVIDERS[$idx]}"
    model="${DRIVER_MODELS[$idx]}"
    FULL_IMAGE="${REGISTRY}/${IMAGE_NAMESPACE}/${provider}/${model}:${TAG}"

    echo ""
    echo "============================================"
    echo "构建 ${name}  →  ${FULL_IMAGE}"
    echo "============================================"

    # If driver.yaml has build_context_extras, create a temp context with those dirs copied in
    yaml_file="${dir}driver.yaml"
    extras=()
    while IFS= read -r extra; do
        [ -n "${extra}" ] && extras+=("${extra}")
    done < <(_parse_yaml_list "${yaml_file}" "build_context_extras")

    BUILD_CTX="${dir}"
    CLEANUP_CTX=""
    if [ ${#extras[@]} -gt 0 ]; then
        BUILD_CTX=$(mktemp -d)
        CLEANUP_CTX="${BUILD_CTX}"
        cp -r "${dir}." "${BUILD_CTX}/"
        for extra in "${extras[@]}"; do
            src="${dir}${extra}"
            if [ -d "${src}" ]; then
                # Extras may live above the driver directory (for example ../../common).
                # Always copy them under their basename so the temporary Docker context
                # cannot escape through a ../ destination.
                extra_dest="$(basename "${extra%/}")"
                cp -r "${src}" "${BUILD_CTX}/${extra_dest}"
            else
                echo "警告：build_context_extras 中的 ${extra} 不存在，跳过"
            fi
        done
    fi

    # Use the builder selected by the active Docker context. Docker Desktop
    # commonly names it desktop-linux; forcing `default` crosses contexts and
    # fails before the build starts.
    docker buildx build \
        --platform linux/arm64 \
        ${NO_CACHE} \
        --build-arg "PYPI_MIRROR=${PYPI_MIRROR}" \
        --build-arg "ROS_APT_MIRROR=${ROS_APT_MIRROR}" \
        --build-arg "UBUNTU_APT_MIRROR=${UBUNTU_APT_MIRROR}" \
        --file "${dir}Dockerfile" \
        --tag "${FULL_IMAGE}" \
        --output=type=docker \
        "${BUILD_CTX}"

    [ -n "${CLEANUP_CTX}" ] && rm -rf "${CLEANUP_CTX}"

    relative_dir="${dir#${SCRIPT_DIR}/}"
    if [ "${relative_dir%/}" = "engineai/t800" ]; then
        echo "[smoke] 验证 T800 ARM64 镜像依赖和 ROS 接口..."
        docker run --rm --platform linux/arm64 --entrypoint /bin/bash "${FULL_IMAGE}" -ce \
            'source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; source /t800_ws/install/setup.bash; command -v pactl; command -v parec; ros2 pkg prefix rmw_cyclonedds_cpp; python3 -c "import main; from audio_msgs.msg import AudioChunk; from interface_protocol.msg import BodyVelCmd, Heartbeat, LinkInfo, MotionStateRequest; from nav_msgs.msg import Odometry; from sensor_msgs.msg import CompressedImage, Image, PointCloud2; import lcm, numpy, yaml"'
    fi

    # Publish only after the locally loaded image passes its architecture and
    # driver-specific smoke checks. A failed smoke must never create a remote tag.
    if ${PUSH_ENABLED}; then
        docker push "${FULL_IMAGE}"
    fi

    BUILT_INDICES+=("${idx}")
    echo "完成：${FULL_IMAGE}"
done

echo ""
echo "全部完成。"

# ── 注册到 Resource Center ──────────────────────────────────────────────────
if ${PUSH_ENABLED} && [ -n "${RESOURCE_CENTER_API_KEY:-}" ]; then
    SYNC_CONFIRM="${RESOURCE_CENTER_SYNC:-}"
    case "${SYNC_CONFIRM}" in
        always|yes|y|Y) SYNC_CONFIRM="y" ;;
        never|no|n|N) SYNC_CONFIRM="n" ;;
        "")
            SYNC_CONFIRM="y"
            if [ -t 0 ] && [ -t 1 ]; then
                printf "\nSync to resource-center (%s)? [Y/n]: " "${RESOURCE_CENTER_URL}"
                read -r SYNC_CONFIRM || SYNC_CONFIRM="y"
            fi
            ;;
        *)
            echo "错误：RESOURCE_CENTER_SYNC 必须是 always/never。" >&2
            exit 2
            ;;
    esac
    if [[ ! "${SYNC_CONFIRM}" =~ ^[Nn] ]]; then
        echo ""
        echo "注册镜像到 Resource Center (${RESOURCE_CENTER_URL})..."
        for idx in "${BUILT_INDICES[@]}"; do
            name="${DRIVER_NAMES[$idx]}"
            img="${DRIVER_IMAGES[$idx]}"
            driver_id="${DRIVER_IDS[$idx]}"
            cat="${DRIVER_CATS[$idx]}"
            port="${DRIVER_PORTS[$idx]}"
            desc="${DRIVER_DESCS[$idx]}"
            hw_provider="${DRIVER_PROVIDERS[$idx]:-}"
            hw_model="${DRIVER_MODELS[$idx]:-}"
            FULL_IMAGE="${REGISTRY}/${IMAGE_NAMESPACE}/${hw_provider}/${hw_model}:${TAG}"

            payload="{
  \"imageRef\": \"${FULL_IMAGE}\",
  \"registryImage\": \"${img}\",
  \"tag\": \"${TAG}\",
  \"category\": \"${cat}\",
  \"hardware_provider\": \"${hw_provider}\",
  \"hardware_model\": \"${hw_model}\",
  \"name\": \"${name}\",
  \"description\": \"${desc}\",
  \"port\": ${port:-null}
}"

            response_file="$(mktemp /tmp/rc_register_resp.XXXXXX)"
            if ! http_code=$(curl -s -o "${response_file}" -w "%{http_code}" \
                    -X POST "${RESOURCE_CENTER_URL}/api/admin/register" \
                    -H "x-api-key: ${RESOURCE_CENTER_API_KEY}" \
                    -H "Content-Type: application/json" \
                    -d "${payload}"); then
                rm -f "${response_file}"
                echo "  ✗ ${name} 注册请求失败" >&2
                exit 1
            fi

            resp="$(cat "${response_file}")"
            rm -f "${response_file}"
            if [ "${http_code}" = "200" ] || [ "${http_code}" = "201" ]; then
                echo "  ✓ ${name}"
                echo "    imageRef : ${FULL_IMAGE}"
                echo "    category : ${cat}"
                [ -n "${hw_provider}" ] && echo "    provider : ${hw_provider}"
                [ -n "${hw_model}" ]    && echo "    model    : ${hw_model}"
                echo "    response : ${resp}"
            else
                echo "  ✗ ${name} 注册失败 (HTTP ${http_code}): ${resp}" >&2
                exit 1
            fi
        done
    else
        echo "跳过同步。"
    fi
fi
