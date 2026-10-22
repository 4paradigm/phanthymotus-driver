#!/usr/bin/env python3
"""Explicit, pinned installation of the optional simulation backend.

Run inside a dedicated Python 3.10–3.12 virtual environment. This script never
installs robot SDKs and is not called from the card or driver lifecycle.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path, PurePosixPath

from upstream import (
    ASSETS, ASSET_RECEIPT, ASSET_REVISION, SOURCE_REVISION, SOURCE_URL,
    Asset, inspect_installation, inspect_source, sha256_file,
)


LICENSE_NOTICE = """Teleopit 根源码为 Apache-2.0，模型仓库标注 MIT；第三方材料另有条款。
其中 lafan_vendor/license.txt 为 CC-BY-NC-ND-4.0，示例 BVH 来自 LAFAN1。
本脚本不替第三方授予商业使用/再分发许可；商业部署前需单独核对组件和数据许可。
不会把这些外部源码、模型或示例提交到 Driver 仓库。"""


def ensure_checkout(root: Path) -> None:
    """Create a pinned checkout, never reset an operator's existing one."""
    if root.exists():
        _, errors = inspect_source(root)
        if errors:
            raise ValueError("；".join(errors))
        return
    root.parent.mkdir(parents=True, exist_ok=True)
    # Stage our own clone so a failed network operation leaves no partial root.
    with tempfile.TemporaryDirectory(prefix=".teleopit-source-", dir=root.parent) as tmp:
        staging = Path(tmp) / "source"
        subprocess.run(["git", "clone", "--no-checkout", "--filter=blob:none",
                        SOURCE_URL, str(staging)], check=True)
        subprocess.run(["git", "-C", str(staging), "checkout", "--detach",
                        SOURCE_REVISION], check=True)
        _, errors = inspect_source(staging)
        if errors:
            raise ValueError("；".join(errors))
        if root.exists():
            raise FileExistsError(f"安装目录已被创建，未覆盖: {root}")
        staging.rename(root)


def download_asset(asset: Asset, cache: Path) -> Path:
    """Download a revision-pinned file and verify size plus SHA-256."""
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / f"{asset.sha256}-{Path(asset.remote_path).name}"
    if target.exists():
        if (target.is_symlink() or not target.is_file()
                or target.stat().st_size != asset.size
                or sha256_file(target) != asset.sha256):
            raise ValueError(f"缓存文件校验失败，未覆盖: {target}")
        return target
    with tempfile.TemporaryDirectory(prefix=".download-", dir=cache) as tmp:
        staged = Path(tmp) / "asset"
        request = urllib.request.Request(asset.url, headers={"User-Agent": "phanthymotus-teleopit-setup/1"})
        with urllib.request.urlopen(request, timeout=60) as response, staged.open("xb") as output:
            count = 0
            while block := response.read(1024 * 1024):
                count += len(block)
                if count > asset.size:
                    raise ValueError(f"下载文件大于固定清单: {asset.name}")
                output.write(block)
        if staged.stat().st_size != asset.size or sha256_file(staged) != asset.sha256:
            raise ValueError(f"下载文件 SHA-256 或大小不匹配: {asset.name}")
        # Atomic no-overwrite publication, including concurrent installers.
        os.link(staged, target)
    return target


def safe_extract(archive: Path, target: Path, *, max_bytes: int = 2_000_000_000) -> None:
    """Extract regular files/directories only; reject links and unsafe paths.

    Do not use extractall: Python 3.10's extraction filter is insufficient and
    tar symlinks/hardlinks can escape a lexically safe member path.
    """
    target.mkdir(parents=True, exist_ok=False)
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        total = 0
        seen = set()
        for member in members:
            name = PurePosixPath(member.name)
            if (name.is_absolute() or ".." in name.parts or "\\" in member.name
                    or ":" in member.name or not (member.isdir() or member.isfile())):
                raise ValueError(f"拒绝不安全的归档成员: {member.name}")
            if str(name) in ("", "."):
                if member.isdir():
                    continue
                raise ValueError("拒绝空归档文件名")
            if name in seen:
                raise ValueError(f"拒绝重复的归档成员: {member.name}")
            seen.add(name)
            total += member.size
            if member.size < 0 or total > max_bytes:
                raise ValueError("归档展开超过大小限制")
        for member in members:
            name = PurePosixPath(member.name)
            if str(name) in ("", "."):
                continue
            destination = target.joinpath(*name.parts)
            if member.isdir():
                destination.mkdir(parents=True, exist_ok=True)
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                source = tar.extractfile(member)
                if source is None:
                    raise ValueError(f"无法读取归档成员: {member.name}")
                with source, destination.open("xb") as output:
                    shutil.copyfileobj(source, output)


def _tree_signature(path: Path) -> dict[str, str]:
    if path.is_symlink():
        raise ValueError(f"不覆盖符号链接: {path}")
    if path.is_file():
        return {".": sha256_file(path)}
    if not path.is_dir():
        raise ValueError(f"不是普通文件或目录: {path}")
    result = {}
    for item in sorted(path.rglob("*")):
        if item.is_symlink():
            raise ValueError(f"不接受资源符号链接: {item}")
        if item.is_file():
            result[item.relative_to(path).as_posix()] = sha256_file(item)
        elif not item.is_dir():
            raise ValueError(f"不接受特殊资源文件: {item}")
    return result


def install_asset(asset: Asset, downloaded: Path, root: Path) -> dict:
    """Place assets without overwriting an existing, differing file/tree."""
    if downloaded.stat().st_size != asset.size or sha256_file(downloaded) != asset.sha256:
        raise ValueError(f"安装输入校验失败: {asset.name}")
    destination = root / asset.local_path
    ancestor = root
    for part in Path(asset.local_path).parts:
        ancestor = ancestor / part
        if ancestor.is_symlink():
            raise ValueError(f"不接受资源目录中的符号链接: {ancestor}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".asset-", dir=destination.parent) as tmp:
        staged = Path(tmp) / "asset"
        if asset.archive:
            safe_extract(downloaded, staged)
        else:
            shutil.copyfile(downloaded, staged)
        signature = _tree_signature(staged)
        if destination.exists() or destination.is_symlink():
            if _tree_signature(destination) != signature:
                raise ValueError(f"现有资源与固定版本不同，未覆盖: {destination}")
        elif asset.archive:
            staged.rename(destination)
        else:
            os.link(staged, destination)
    return {"sha256": asset.sha256, "size": asset.size,
            "local_path": asset.local_path, "files": signature}


def install_dependencies(root: Path, *, pico: bool = False) -> None:
    if not (3, 10) <= sys.version_info[:2] <= (3, 12):
        raise ValueError("请使用 Python 3.10–3.12 虚拟环境安装已验证依赖")
    if sys.prefix == sys.base_prefix:
        raise ValueError("拒绝修改全局 Python；请先创建并激活专用虚拟环境")
    if importlib.util.find_spec("pip") is None:
        raise ValueError("虚拟环境缺少 pip；请用 uv venv --seed 或 python -m venv 创建环境")
    pip = [sys.executable, "-m", "pip", "install"]
    if platform.system() == "Linux" and platform.machine().lower() in ("x86_64", "amd64"):
        # Avoid downloading CUDA dependencies for this CPU-only simulation driver.
        subprocess.run(pip + ["--index-url", "https://download.pytorch.org/whl/cpu",
                              "torch==2.2.2"], check=True)
    requirements = Path(__file__).with_name("requirements-sim.txt")
    subprocess.run(pip + ["-r", str(requirements)], check=True)
    subprocess.run(pip + ["--no-deps", "-e", str(root)], check=True)
    if pico:
        subprocess.run(pip + ["-r", str(Path(__file__).with_name("requirements-pico.txt"))], check=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path, help="专用 Teleopit checkout 目录")
    parser.add_argument("--cache", type=Path, help="下载缓存，默认 ROOT.parent/teleopit-asset-cache")
    parser.add_argument("--check", action="store_true", help="只检查，不安装或访问网络")
    parser.add_argument("--assets-only", action="store_true", help="只准备源码/资源，不修改 Python 环境")
    parser.add_argument("--pico", action="store_true", help="另装固定 pico-bridge；不装真机 SDK")
    parser.add_argument("--with-gmr", action="store_true", help="另装其他机器人 GMR 资源（约 365 MB；G1 不需要）")
    parser.add_argument("--accept-third-party-licenses", action="store_true", help="已阅读第三方许可说明，确认本地安装用途合规")
    args = parser.parse_args(argv)
    root = args.root.expanduser().resolve()
    if args.check:
        report = inspect_installation(root, source="pico" if args.pico else "bvh")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ready"] else 1
    print(LICENSE_NOTICE)
    if not args.accept_third_party_licenses:
        parser.error("请阅读以上许可说明后，显式提供 --accept-third-party-licenses")
    cache = (args.cache or root.parent / "teleopit-asset-cache").expanduser().resolve()
    try:
        ensure_checkout(root)
        receipt = {"source_revision": SOURCE_REVISION, "asset_revision": ASSET_REVISION, "assets": {}}
        for asset in ASSETS:
            if asset.name == "gmr" and not args.with_gmr:
                continue
            print(f"准备 {asset.name} ({asset.size / 1_000_000:.1f} MB): {asset.url}", flush=True)
            receipt["assets"][asset.name] = install_asset(asset, download_asset(asset, cache), root)
        # This generated receipt is owned solely by this installer, not source or assets.
        receipt_path = root / ASSET_RECEIPT
        if receipt_path.is_symlink():
            raise ValueError(f"不覆盖符号链接: {receipt_path}")
        receipt_path.write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
        if not args.assets_only:
            install_dependencies(root, pico=args.pico)
        report = inspect_installation(root, source="pico" if args.pico else "bvh")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ready"] else 1
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f"安装失败（未覆盖已有源码/资源）: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
