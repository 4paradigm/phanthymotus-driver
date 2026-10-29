"""Verify G1 offline modules against the Dockerfile's flat Python layout."""
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
G1 = ROOT / "unitree" / "g1"
MODULES = ("loco_descriptor.py", "loco_admission.py", "base_owner.py")


def flat_layout(tmp_path):
    common = tmp_path / "common"
    common.mkdir()
    shutil.copy2(ROOT / "common" / "__init__.py", common)
    shutil.copytree(ROOT / "common" / "control", common / "control",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    for name in MODULES:
        shutil.copy2(G1 / name, tmp_path / name)


def isolated_python(tmp_path, code):
    return subprocess.run([sys.executable, "-I", "-B", "-c",
                           "import sys; sys.path.insert(0, sys.argv[1]); " + code,
                           str(tmp_path)], cwd=tmp_path, capture_output=True, text=True)


def test_dockerfile_copies_offline_modules_to_work():
    lines = (G1 / "Dockerfile").read_text().splitlines()
    copies = [shlex.split(line) for line in lines if line.startswith("COPY ")]
    for name in MODULES:
        assert any(name in entry[1:-1] and entry[-1] in ("/work/", f"/work/{name}")
                   for entry in copies), name
    assert any(entry[1:] == ["common/", "/work/common/"] for entry in copies)
    assert "ENV PYTHONPATH=/work" in lines


def test_flat_layout_imports_without_ros(tmp_path):
    flat_layout(tmp_path)
    result = isolated_python(tmp_path, "import loco_admission, loco_descriptor, base_owner; "
                             "assert loco_admission.validate_loco_descriptor is "
                             "loco_descriptor.validate_loco_descriptor; "
                             "assert 'rclpy' not in sys.modules")
    assert result.returncode == 0, result.stderr


def test_package_layout_uses_package_descriptor():
    from unitree.g1 import loco_admission, loco_descriptor
    assert loco_admission.validate_loco_descriptor is loco_descriptor.validate_loco_descriptor


def test_dependency_import_failure_is_not_swallowed(tmp_path):
    flat_layout(tmp_path)
    (tmp_path / "loco_descriptor.py").write_text("import missing_g1_test_dependency\n")
    result = isolated_python(tmp_path, "import loco_admission")
    assert result.returncode != 0
    assert "No module named 'missing_g1_test_dependency'" in result.stderr
