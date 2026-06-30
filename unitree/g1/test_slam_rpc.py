#!/usr/bin/env python3
"""
Standalone SLAM RPC test — run directly on robot to diagnose timeout issues.
Usage: python3 test_slam_rpc.py eth0
"""
import sys
import time
import json

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.slam.slam_client import SlamClient

def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    print(f"[test] Initializing DDS on {iface}...")
    ChannelFactoryInitialize(0, iface)
    print("[test] DDS initialized")

    print("[test] Creating SlamClient...")
    client = SlamClient()
    client.SetTimeout(5.0)
    client.Init()
    print("[test] SlamClient ready, timeout=5s")

    # Wait for DDS discovery
    time.sleep(1.0)
    print("[test] DDS discovery wait done\n")

    # --- Test 1: StartMapping ---
    print("=" * 50)
    print("[test] Calling StartMapping...")
    t0 = time.monotonic()
    code, resp = client.StartMapping()
    elapsed = time.monotonic() - t0
    print(f"[test] StartMapping → code={code}, elapsed={elapsed:.3f}s")
    print(f"[test] response: {resp}")

    if code != 0:
        print(f"\n[test] StartMapping FAILED with code={code}")
        print("[test] Trying with longer timeout (10s)...")
        client.SetTimeout(10.0)
        t0 = time.monotonic()
        code2, resp2 = client.StartMapping()
        elapsed2 = time.monotonic() - t0
        print(f"[test] StartMapping (10s) → code={code2}, elapsed={elapsed2:.3f}s")
        print(f"[test] response: {resp2}")
        client.SetTimeout(5.0)

        if code2 != 0:
            print("\n[test] Still failing. Testing GetServerApiVersion (simpler RPC)...")
            t0 = time.monotonic()
            code3, ver = client.GetServerApiVersion()
            elapsed3 = time.monotonic() - t0
            print(f"[test] GetServerApiVersion → code={code3}, elapsed={elapsed3:.3f}s, version={ver}")
            return

    print("\n[test] Mapping started. Waiting 3s before stopping...")
    time.sleep(3.0)

    # --- Test 2: StopMapping ---
    print("=" * 50)
    pcd_path = "/tmp/test_slam_rpc.pcd"
    print(f"[test] Calling StopMapping (pcd={pcd_path})...")
    client.SetTimeout(10.0)
    t0 = time.monotonic()
    code, resp = client.StopMapping(pcd_path)
    elapsed = time.monotonic() - t0
    print(f"[test] StopMapping → code={code}, elapsed={elapsed:.3f}s")
    print(f"[test] response: {resp}")

    print("\n[test] Done.")

if __name__ == "__main__":
    main()
