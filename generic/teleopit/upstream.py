"""Lightweight, read-only checks for the optional Teleopit installation.

This module never imports the simulator, installs packages, or contacts hardware.
Downloads are available only through the operator-invoked setup_teleopit.py.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path


SOURCE_URL = "https://github.com/BotRunner64/Teleopit.git"
SOURCE_VERSION = "0.5.0"
SOURCE_REVISION = "f9263865c581802ad531854b8e547e2403a945f3"
ASSET_REPOSITORY = "12e21/Teleopit-models"
ASSET_REVISION = "94cf996444fea6894b87c28e86606cd4c2f1408f"
DEFAULT_POLICY = "ckpt/track_g1.onnx"
DEFAULT_BVH = "data/sample_bvh/aiming1_subject1.bvh"
ROBOT_XML = "assets/robots/unitree_g1/g1_29dof.xml"
ASSET_RECEIPT = ".phanthymotus-teleopit-assets.json"


@dataclass(frozen=True)
class Asset:
    name: str
    remote_path: str
    local_path: str
    sha256: str
    size: int
    archive: bool = False

    @property
    def url(self) -> str:
        return (f"https://huggingface.co/{ASSET_REPOSITORY}/resolve/"
                f"{ASSET_REVISION}/{self.remote_path}")


# Digests and sizes are the Git LFS object identities from the immutable model
# repository revision above, not hashes fetched dynamically at installation time.
ASSETS = (
    Asset("policy", "checkpoints/track_g1.onnx", DEFAULT_POLICY,
          "1ebd341d9193e1c49a986450f6043ba1a9473ad46636ce0bcb1c7755c856e0de",
          14_077_633),
    Asset("robots", "archives/robot_assets.tar.gz", "assets/robots",
          "fb5f1aeec3c57be6b26533c9a9aad0d095048a5fe8aa3c6915d8f6a67c35d4fc",
          27_765_695, True),
    Asset("bvh", "archives/sample_bvh.tar.gz", "data/sample_bvh",
          "c6be528b094c4140285a3e7449303f5d65210223f16b896036eb86071505730e",
          2_539_053, True),
    # Not needed by G1: v0.5.0 GMR reuses the canonical assets/robots G1 model.
    Asset("gmr", "archives/gmr_assets.tar.gz", "teleopit/retargeting/gmr/assets",
          "538f52afaccbbe68bf35ec2d3289dbdb296c9270551b022f2a937a11ff841b04",
          365_137_251, True),
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def inspect_source(root: Path) -> tuple[str | None, list[str]]:
    """Reject other revisions and locally modified tracked upstream code."""
    root = root.expanduser().resolve()
    try:
        def git(*args: str) -> str:
            return subprocess.run(
                ["git", "-C", str(root), *args], capture_output=True, text=True,
                check=True, timeout=10,
            ).stdout.strip()

        if Path(git("rev-parse", "--show-toplevel")).resolve() != root:
            return None, ["TELEOPIT_ROOT 必须指向独立的 Teleopit 源码仓库根目录"]
        revision = git("rev-parse", "HEAD")
        errors = []
        if revision != SOURCE_REVISION:
            errors.append(f"Teleopit 源码版本不匹配；要求 {SOURCE_REVISION}")
        if git("status", "--porcelain", "--untracked-files=no"):
            errors.append("Teleopit 已跟踪源码存在本地修改；请使用干净的固定版本 checkout")
        return revision, errors
    except (OSError, subprocess.SubprocessError):
        return None, ["未找到可校验的 Teleopit Git checkout；请运行 setup_teleopit.py"]


def _resolve_asset(root: Path, value: str | Path | None, default: str) -> Path:
    path = Path(value or default).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def inspect_installation(
    root: Path, policy_path: str | Path | None = None,
    bvh_path: str | Path | None = None, source: str = "bvh",
) -> dict:
    """Inspect files/provenance without importing any optional Python package.

    ``ready`` concerns source and assets only. The backend performs imports,
    ONNX shape checks and simulator construction in its cancellable child process.
    Custom BVH and ONNX paths are operator-owned; they are never downloaded here.
    """
    root = Path(root).expanduser().resolve()
    revision, errors = inspect_source(root)
    warnings = []
    policy = _resolve_asset(root, policy_path, DEFAULT_POLICY)
    bvh = _resolve_asset(root, bvh_path, DEFAULT_BVH)
    xml = root / ROBOT_XML
    required = {"policy": policy, "robot_xml": xml}
    if source == "bvh":
        required["bvh"] = bvh
    elif source != "pico":
        errors.append("source 必须为 bvh 或 pico")
    required["config"] = root / "teleopit/configs/default.yaml"
    required["retarget_config"] = root / (
        "teleopit/retargeting/gmr/ik_configs/"
        + ("pico_bridge_to_g1.json" if source == "pico" else "bvh_lafan1_to_g1.json")
    )
    for name, path in required.items():
        try:
            exists = path.is_file() and path.stat().st_size > 0
        except OSError:
            exists = False
        if not exists:
            errors.append(f"缺少 {name}: {path}")

    policy_verified = False
    if policy == (root / DEFAULT_POLICY).resolve() and policy.is_file():
        try:
            policy_verified = (policy.stat().st_size == ASSETS[0].size
                               and sha256_file(policy) == ASSETS[0].sha256)
        except OSError:
            policy_verified = False
        if not policy_verified:
            errors.append("默认 G1 ONNX 策略 SHA-256 不匹配；请勿混用旧策略")
    elif policy.is_file():
        warnings.append("使用自定义 ONNX；其来源未由安装清单验证，仍需后端维度校验")

    receipt = None
    try:
        receipt = json.loads((root / ASSET_RECEIPT).read_text(encoding="utf-8"))
        if (not isinstance(receipt, dict) or receipt.get("asset_revision") != ASSET_REVISION
                or receipt.get("source_revision") != SOURCE_REVISION
                or not isinstance(receipt.get("assets"), dict)):
            warnings.append("资源安装凭据版本不匹配；请用安装脚本重新校验")
            receipt = None
    except (OSError, ValueError):
        warnings.append("未发现有效资源安装凭据；源码及策略已单独检查，网格由 MuJoCo 加载校验")
    # Detect replacement of the selected canonical XML/BVH after installation.
    # Meshes are not rehashed on every preflight; full trees are compared when the
    # explicit installer is re-run. MuJoCo validates references and dimensions.
    selected = {"robots": (xml, "unitree_g1/g1_29dof.xml")}
    if source == "bvh" and bvh == (root / DEFAULT_BVH).resolve():
        selected["bvh"] = (bvh, "aiming1_subject1.bvh")
    if receipt:
        for name, (path, relative) in selected.items():
            asset_receipt = receipt["assets"].get(name, {})
            files = asset_receipt.get("files", {}) if isinstance(asset_receipt, dict) else {}
            expected = files.get(relative) if isinstance(files, dict) else None
            if not isinstance(expected, str):
                warnings.append(f"安装凭据缺少 {name} 的文件校验记录")
            elif path.is_file():
                try:
                    if sha256_file(path) != expected:
                        errors.append(f"安装后 {name} 资源已改变；请重新校验: {path}")
                except OSError:
                    errors.append(f"无法读取 {name} 资源: {path}")
    return {
        "ready": not errors,
        "errors": errors,
        "warnings": warnings,
        "source_version": SOURCE_VERSION,
        "source_revision": revision,
        "expected_source_revision": SOURCE_REVISION,
        "asset_revision": ASSET_REVISION,
        "policy_verified": policy_verified,
        "paths": {"root": str(root), "policy": str(policy), "bvh": str(bvh),
                  "robot_xml": str(xml)},
        "assets": {entry.name: {"sha256": entry.sha256, "size": entry.size,
                                "local_path": entry.local_path}
                   for entry in ASSETS if entry.name != "gmr"},
        "receipt_present": receipt is not None,
    }
