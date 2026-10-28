"""Build-time status schema generation; require exact shipped XTypes metadata."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess

IDL = r"""
module RobotStatus {
  struct ImuData {
    double ori[4]; double ori_cov[9];
    double angular_vel[3]; double angular_vel_cov[9];
    double linear_acc[3]; double linear_acc_cov[9];
  };
  struct JoyData { double axes[2]; long button[14]; };
  struct MotorState {
    double pos; double vel; double tau;
    unsigned short motor_id; unsigned short error; long temperature;
  };
  struct BmsState {
    unsigned short battery_temp; unsigned short battery_alarm;
    unsigned short battery_soc; unsigned short battery_soh;
  };
  struct MotorStateArray { sequence<MotorState, 21> motorstates; long long timestamp; };
  struct StatusData {
    ImuData imudata; JoyData joydata; BmsState bmsdata;
    unsigned short workmode; MotorStateArray motorstatearray;
  };
};
"""


def schema_blobs(text):
    pattern = (r"TopicTraits<::RobotStatus::(\w+)>::type_(map|info)_blob\(\)\s*"
               r"\{(.*?)return blob;\s*\}")
    return {(name, kind): bytes(int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]{2})", body))
            for name, kind, body in re.findall(pattern, text, re.DOTALL)}


def generate(sdk_root, runtime_root, output):
    arch = {"x86_64": "x86_64", "aarch64": "aarch64"}[platform.machine()]
    runtime = Path(runtime_root) / "dds" / arch
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    compiler = output / "offline-idlc"
    shutil.copyfile(runtime / "bin/idlc", compiler)
    compiler.chmod(0o700)
    idl = output / "Robot_status.idl"
    idl.write_text(IDL)
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = str((runtime / "lib").resolve()) + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    subprocess.run([str(compiler.resolve()), "-l", "cxx", "-n", "false", "-x", "final",
                    "-o", str(output.resolve()), str(idl.resolve())], env=env, check=True, timeout=30)
    expected = schema_blobs((Path(sdk_root) / "include/Robot_status.hpp").read_text())
    actual = schema_blobs((output / "Robot_status.hpp").read_text())
    if len(expected) != 12 or expected != actual:
        raise ValueError("SCHEMA_MISMATCH: generated status metadata differs from bundled SDK")
    print("payload_mass DDS schema matches all 12 bundled XTypes blobs", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdk_root", type=Path)
    parser.add_argument("runtime_root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.sdk_root, args.runtime_root, args.output)
