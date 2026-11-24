"""Verify PiPER vision deployment over MCP without submitting a motion."""

import argparse
import json
import urllib.request


def call(endpoint, action, **params):
    body = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {"name": "visio_pick_and_drop", "arguments": {"action": action, **params}}}
    request = urllib.request.Request(endpoint, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=5) as response:
        result = json.load(response)
    if "error" in result:
        raise RuntimeError(result["error"]["message"])
    return json.loads(result["result"]["content"][0]["text"])


def normalize_pixel(pixel, observation):
    width, height = observation["width"], observation["height"]
    u, v = pixel
    if not (0 <= u < width and 0 <= v < height):
        raise ValueError(f"像素 ({u}, {v}) 超出当前图像尺寸 {width}×{height}")
    return 2 * u / width - 1, 2 * v / height - 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="http://127.0.0.1:15741/mcp")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--point", type=float, nargs=2, metavar=("X", "Y"),
                           help="Run observe and plan_pick at normalized coordinates [-1,1]")
    selection.add_argument("--pixel", type=int, nargs=2, metavar=("U", "V"),
                           help="Run observe and plan_pick at a pixel of the original RGB image")
    args = parser.parse_args()
    try:
        report = {"info": call(args.endpoint, "info")}
        if args.point is not None or args.pixel is not None:
            observation = call(args.endpoint, "observe")
            x, y = args.point if args.pixel is None else normalize_pixel(args.pixel, observation)
            report["observation"] = observation
            report["plan"] = call(args.endpoint, "plan_pick", observation_id=observation["observation_id"],
                                  pick_x=x, pick_y=y)
            if args.pixel is not None and report["plan"]["pick_pixel"] != args.pixel:
                raise ValueError("卡片实际读取的像素与请求选点不一致")
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
