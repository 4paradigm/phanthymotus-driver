#!/usr/bin/env bash
# build.sh — 构建硬件驱动镜像并推送到镜像仓库
#
# 用法：bash build.sh [driver_dir...]
#   不传参数时显示交互式多选框
#   直接传目录名时跳过选择（CI 用）
#
# 依赖：
#   - 每个驱动目录下需有 driver.yaml 和 Dockerfile
#   - python3（解析 YAML）或 yq（可选）
#   - 镜像仓库配置从 ../phanthy-motus/deploy/.env 读取
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOTUS_ROOT="$(cd "${SCRIPT_DIR}/../phanthy-motus" && pwd)"
ENV_FILE="${MOTUS_ROOT}/deploy/.env"
DRIVERS_YAML="${MOTUS_ROOT}/deploy/core/config/drivers.yaml"

# ── 加载 .env ──────────────────────────────────────────────────────────────
if [ -f "${ENV_FILE}" ]; then
    source "${ENV_FILE}"
fi

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
    # 直接传目录参数（CI 模式）
    for arg in "$@"; do
        for i in "${!DRIVER_DIRS[@]}"; do
            if [[ "${DRIVER_DIRS[$i]}" == *"${arg}"* ]]; then
                SELECTED_INDICES+=("$i")
            fi
        done
    done
else
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
    echo "未选择任何驱动，退出。"
    exit 0
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
TAG="release.${DATE}.${COMMIT}"

echo ""
echo "版本 tag：${TAG}"
echo "目标仓库：${REGISTRY}/${IMAGE_NAMESPACE}/"
echo ""

# ── 登录 & QEMU ───────────────────────────────────────────────────────────
if ${PUSH_ENABLED}; then
    echo "${REGISTRY_PASSWORD}" | docker login "${REGISTRY}" -u "${REGISTRY_USER}" --password-stdin
fi
docker run --privileged --rm mirror.ccs.tencentyun.com/tonistiigi/binfmt --install arm64

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
                cp -r "${src}" "${BUILD_CTX}/${extra}"
            else
                echo "警告：build_context_extras 中的 ${extra} 不存在，跳过"
            fi
        done
    fi

    docker buildx build \
        --builder default \
        --platform linux/arm64 \
        --file "${dir}Dockerfile" \
        --tag "${FULL_IMAGE}" \
        $(${PUSH_ENABLED} && echo "--push" || echo "--output=type=docker") \
        "${BUILD_CTX}"

    [ -n "${CLEANUP_CTX}" ] && rm -rf "${CLEANUP_CTX}"

    BUILT_INDICES+=("${idx}")
    echo "完成：${FULL_IMAGE}"
done

# ── 更新 drivers.yaml ─────────────────────────────────────────────────────
if [ ${#BUILT_INDICES[@]} -gt 0 ] && [ -f "${DRIVERS_YAML}" ]; then
    echo ""
    echo "更新 ${DRIVERS_YAML} ..."
    python3 - "${DRIVERS_YAML}" "${REGISTRY}" "${IMAGE_NAMESPACE}" "${TAG}" \
        "${#BUILT_INDICES[@]}" \
        "${BUILT_INDICES[@]/#/}" \
        "${DRIVER_IDS[@]}" \
        "${DRIVER_NAMES[@]}" \
        "${DRIVER_CATS[@]}" \
        "${DRIVER_IMAGES[@]}" \
        "${DRIVER_PORTS[@]}" \
        "${DRIVER_MCPS[@]}" \
        "${DRIVER_DESCS[@]}" \
        "${DRIVER_PROVIDERS[@]}" \
        "${DRIVER_MODELS[@]}" <<'PYEOF'
import sys, re

yaml_path  = sys.argv[1]
registry   = sys.argv[2]
namespace  = sys.argv[3]
tag        = sys.argv[4]
n_built    = int(sys.argv[5])

built_indices = [int(x) for x in sys.argv[6:6+n_built]]
ids    = sys.argv[6+n_built:]
# ids/names/cats/images/ports/mcps/descs/providers/models each has len = total drivers
# split equally (9 fields)
total = len(ids) // 9
driver_ids       = ids[0*total:1*total]
driver_names     = ids[1*total:2*total]
driver_cats      = ids[2*total:3*total]
driver_imgs      = ids[3*total:4*total]
driver_ports     = ids[4*total:5*total]
driver_mcps      = ids[5*total:6*total]
driver_descs     = ids[6*total:7*total]
driver_providers = ids[7*total:8*total]
driver_models    = ids[8*total:9*total]

with open(yaml_path) as f:
    content = f.read()

for idx in built_indices:
    image_name = driver_imgs[idx]
    hw_provider = driver_providers[idx]
    hw_model    = driver_models[idx]
    full_image = f'{registry}/{namespace}/{hw_provider}/{hw_model}:{tag}'
    driver_id  = driver_ids[idx]
    name       = driver_names[idx]
    category   = driver_cats[idx]
    port       = driver_ports[idx]
    mcp_url    = driver_mcps[idx]
    desc       = driver_descs[idx]

    # Try to update existing entry's image field
    pattern = rf'(- id: {re.escape(driver_id)}.*?image: )[^\n]+'
    replacement = rf'\g<1>{full_image}'
    new_content, n = re.subn(pattern, replacement, content, flags=re.DOTALL)

    if n > 0:
        content = new_content
        print(f'  更新 {driver_id}: image = {full_image}')
    else:
        # Append new entry before last newline
        entry = f'''
  - id: {driver_id}
    name: {name}
    category: {category}
    registry_image: {image_name}
    image: {full_image}
    port: {port}
    mcp_url: "{mcp_url}"
    description: "{desc}"
'''
        # Insert after "drivers:" line or at end
        if 'drivers:' in content:
            content = content.rstrip('\n') + '\n' + entry
        else:
            content = 'drivers:\n' + entry
        print(f'  新增 {driver_id}: {full_image}')

with open(yaml_path, 'w') as f:
    f.write(content)
print('drivers.yaml 已更新')
PYEOF
fi

echo ""
echo "全部完成。"

# ── 注册到 Resource Center ──────────────────────────────────────────────────
if ${PUSH_ENABLED} && [ -n "${RESOURCE_CENTER_API_KEY:-}" ]; then
    SYNC_CONFIRM="y"
    if [ -t 0 ] || [ -e /dev/tty ]; then
        printf "\nSync to resource-center (%s)? [Y/n]: " "${RESOURCE_CENTER_URL}" >/dev/tty
        read -r SYNC_CONFIRM </dev/tty || SYNC_CONFIRM="y"
    fi
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

            http_code=$(curl -s -o /tmp/rc_register_resp.json -w "%{http_code}" \
                -X POST "${RESOURCE_CENTER_URL}/api/admin/register" \
                -H "x-api-key: ${RESOURCE_CENTER_API_KEY}" \
                -H "Content-Type: application/json" \
                -d "${payload}")

            resp="$(cat /tmp/rc_register_resp.json)"
            if [ "${http_code}" = "200" ] || [ "${http_code}" = "201" ]; then
                echo "  ✓ ${name}"
                echo "    imageRef : ${FULL_IMAGE}"
                echo "    category : ${cat}"
                [ -n "${hw_provider}" ] && echo "    provider : ${hw_provider}"
                [ -n "${hw_model}" ]    && echo "    model    : ${hw_model}"
                echo "    response : ${resp}"
            else
                echo "  ✗ ${name} 注册失败 (HTTP ${http_code}): ${resp}"
            fi
        done
    else
        echo "跳过同步。"
    fi
fi
