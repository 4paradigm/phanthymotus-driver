#!/usr/bin/env python3
"""Run ONLY on a trusted operator workstation, never in Core/Driver/Agent.

Create an Ed25519 keypair once, install only the PUBLIC key on the robot, then
sign a fresh request copied from prepare_authorization. Private keys are never
uploaded to MCP, included in a Solution, or installed on the robot.
"""

import argparse
import base64
import getpass
import json
import os
from pathlib import Path


def canonical(payload):
    return json.dumps(payload, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True, allow_nan=False).encode("ascii")


def main():
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--private-key", type=Path, required=True)
    parser.add_argument("--generate", action="store_true")
    parser.add_argument("--request", type=Path)
    parser.add_argument("--valid-for-s", type=int, default=120)
    args = parser.parse_args()
    # A TTY is an accident guard. The separate signing key is the authority.
    # Separate text streams: Python's r+ wrapper requires seekability, which a
    # real terminal deliberately does not have.
    with open("/dev/tty", "r") as terminal_input, open("/dev/tty", "w") as terminal:
        if not os.isatty(terminal_input.fileno()) or not os.isatty(terminal.fileno()):
            raise SystemExit("An interactive operator terminal is required")
        if args.generate:
            key = Ed25519PrivateKey.generate()
            password = getpass.getpass("Encrypt the new operator key with a passphrase: ", stream=terminal)
            if not password or password != getpass.getpass("Repeat passphrase: ", stream=terminal):
                raise SystemExit("A matching nonempty passphrase is required")
            data = key.private_bytes(serialization.Encoding.PEM,
                                     serialization.PrivateFormat.PKCS8,
                                     serialization.BestAvailableEncryption(password.encode()))
            fd = os.open(args.private_key, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "wb") as stream:
                stream.write(data)
            public = args.private_key.with_suffix(".pub.pem")
            with public.open("xb") as stream:
                stream.write(key.public_key().public_bytes(
                    serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo))
            print("Created encrypted operator key and public key:", public)
            return
        if args.request is None or not 1 <= args.valid_for_s <= 300:
            raise SystemExit("--request and a lifetime from 1 to 300 seconds are required")
        request = json.loads(args.request.read_text())
        fields = {"schema", "device", "process_nonce", "request_id", "session_id", "resource"}
        if (not isinstance(request, dict) or set(request) != fields | {"max_valid_for_s"}
                or request["schema"] != "motus.bumi.sound-facing.authorization/1"
                or request["resource"] != "chassis"
                or any(not isinstance(request[k], str) or not 1 <= len(request[k]) <= 256 for k in fields)):
            raise SystemExit("Invalid authorization request")
        payload = {k: request[k] for k in fields}
        payload["valid_for_s"] = args.valid_for_s
        terminal.write(json.dumps(payload, indent=2) + "\n")
        terminal.write("Authorize this device/chassis session to track sound? Type APPROVE: ")
        terminal.flush()
        if terminal_input.readline().strip() != "APPROVE":
            raise SystemExit("Not approved")
        password = getpass.getpass("Operator key passphrase: ", stream=terminal)
        key = serialization.load_pem_private_key(args.private_key.read_bytes(), password=password.encode())
        if not isinstance(key, Ed25519PrivateKey):
            raise SystemExit("An Ed25519 key is required")
        print(json.dumps({"payload": payload, "signature": base64.b64encode(
            key.sign(canonical(payload))).decode("ascii")}, separators=(",", ":")))


if __name__ == "__main__":
    main()
