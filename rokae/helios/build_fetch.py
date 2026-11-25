"""Build-time download helper: try each URL in turn, print progress, verify SHA256.

Usage: python3 build_fetch.py DEST SHA256 URL [FALLBACK_URL ...]
Each attempt uses a socket timeout so a stalled mirror fails over instead of hanging,
and progress lines keep the build log alive.
"""
import hashlib
import sys
import urllib.request

CHUNK = 1 << 20
TIMEOUT_S = 60


def fetch(dest, url):
    digest = hashlib.sha256()
    done, next_report = 0, 0
    with urllib.request.urlopen(url, timeout=TIMEOUT_S) as response, open(dest, "wb") as out:
        total = int(response.headers.get("Content-Length") or 0)
        while True:
            chunk = response.read(CHUNK)
            if not chunk:
                break
            out.write(chunk)
            digest.update(chunk)
            done += len(chunk)
            if done >= next_report:
                print(f"  {done >> 20} MiB" + (f" / {total >> 20} MiB" if total else ""), flush=True)
                next_report += 8 * CHUNK
    return digest.hexdigest()


def main(dest, sha256, *urls):
    for url in urls:
        if not url:
            continue
        print(f"fetch {url}", flush=True)
        try:
            got = fetch(dest, url)
        except Exception as exc:
            print(f"  failed: {type(exc).__name__}: {exc}", flush=True)
            continue
        if got == sha256:
            print("  sha256 ok", flush=True)
            return 0
        print(f"  sha256 mismatch: {got}", flush=True)
    print("all sources failed", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
