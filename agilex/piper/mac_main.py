#!/usr/bin/env python3
"""Run the control card and optional camera on Mac, without ROS or Docker."""
from common import logsafe
logsafe.install()

import argparse
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from urllib.parse import urlparse, parse_qs

from agilex.piper.arm_control import ArmControlPlugin as PiperPlugin
from common.vendor_runtime import DriverBundle, make_handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",type=int,default=15741)
    parser.add_argument("--usb-index",type=int,default=0)
    parser.add_argument("--data-dir",type=Path,default=Path.home()/".local/share/piper")
    args = parser.parse_args()
    plugin = PiperPlugin({"can_interface":f"gs_usb_{args.usb_index}", "can_transport":"gs_usb",
                          "usb_index":args.usb_index,"data_dir":str(args.data_dir.expanduser()),
                          "camera":{"warmup_seconds":3,"min_contrast":8}})
    bundle = DriverBundle([plugin])
    base = make_handler(lambda:bundle,"agilex-piper-mac","agilex-piper-driver")

    class Handler(base):
        def do_GET(self):
            path = urlparse(self.path)
            if path.path == "/":
                body = Path(__file__).with_name("mac_console.html").read_bytes()
                content_type = "text/html; charset=utf-8"
            elif path.path == "/card-controls.js":
                body = Path(__file__).with_name("card_controls.js").read_bytes()
                content_type = "text/javascript; charset=utf-8"
            elif path.path == "/photo":
                name = parse_qs(path.query).get("name",[""])[0]
                if not (name.startswith("rgb_") and name.endswith(".jpg") and Path(name).name == name):
                    self.send_json(400,{"error":"invalid image name"})
                    return
                try:
                    body = (plugin.data_dir/name).read_bytes()
                except OSError:
                    self.send_json(404,{"error":"image not found"})
                    return
                content_type = "image/jpeg"
            else:
                return super().do_GET()
            self.send_response(200)
            self.send_header("Content-Type",content_type)
            self.send_header("Content-Length",str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    bundle.start_all()
    print(json.dumps({"console":f"http://127.0.0.1:{args.port}/", "motion_enabled":plugin.motion_enabled,
                      "data_dir":str(plugin.data_dir),"dds_available":False},ensure_ascii=False),flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bundle.stop_all()


if __name__ == "__main__":
    main()
