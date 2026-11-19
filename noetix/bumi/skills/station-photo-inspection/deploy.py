"""Send the native skill installer over SSH. OpenSSH prompts you for the password."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

import install_skill


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="Robot SSH host")
    parser.add_argument("--user", default="noetix")
    parser.add_argument("--port", type=int, default=2222)
    parser.add_argument("--container")
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()
    skill = json.loads(Path(__file__).with_name("skill.json").read_text(encoding="utf-8"))
    install_skill.validate_skill(skill)
    remote = ["python3", "-X", "utf8", "-", "--auto"]
    if args.container:
        remote.extend(["--container", args.container])
    if args.replace:
        remote.append("--replace")
    print("Install: " + skill["name"] + " -> " + args.user + "@" + args.host, flush=True)
    print("Enter the SSH password in this terminal when OpenSSH asks. It is not saved.", flush=True)
    try:
        result = subprocess.run(["ssh", "-p", str(args.port), "-o", "ConnectTimeout=10",
                                 args.user + "@" + args.host, shlex.join(remote)],
                                input=install_skill.bundle_source(skill), text=True, encoding="utf-8")
        if result.returncode == 0:
            print("Installed. Refresh Dashboard -> Skills -> Installed, then check the skill is active.")
        return result.returncode
    except OSError as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
