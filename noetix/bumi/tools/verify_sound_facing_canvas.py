"""Read-only Chromium inspection of the real isolated Core Canvas.

Tool start/stop and authorization are exercised by verify_sound_facing_core.py.
This helper never starts the project, enables following or executes an action.
"""
import argparse
import json
from pathlib import Path
from urllib.parse import urlparse


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',required=True)
    parser.add_argument('--token-file',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--chromium',required=True)
    parser.add_argument("--three-dir",type=Path,help="Cached Three.js 0.164 directory; keeps the browser offline")
    args=parser.parse_args()
    if urlparse(args.base).hostname not in ('127.0.0.1','localhost'):parser.error('loopback Core only')
    token=next(line.split('=',1)[1] for line in args.token_file.read_text().splitlines() if line.startswith('ACCESS_TOKEN='))
    args.output.mkdir(exist_ok=False)
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser=pw.chromium.launch(executable_path=args.chromium,headless=True)
        context=browser.new_context(viewport=dict(width=1440,height=1050))
        context.add_init_script('localStorage.setItem("phanthy_access_token",'+json.dumps(token)+');')
        def route_request(route):
            url=route.request.url
            if url.startswith(args.base+'/'):
                route.continue_();return
            prefix='https://cdn.jsdelivr.net/npm/three@0.164/'
            if args.three_dir and url.startswith(prefix):
                root=args.three_dir.resolve()
                asset=(root/url[len(prefix):]).resolve()
                if root in asset.parents and asset.is_file():
                    route.fulfill(path=str(asset),content_type='text/javascript',headers={'Access-Control-Allow-Origin':'*'});return
            route.abort()
        context.route('**/*',route_request)
        page=context.new_page()
        errors=[];page.on('pageerror',lambda error:errors.append(str(error)))
        try:
            page.goto(args.base+'/',wait_until='domcontentloaded')
            card=page.locator('[data-card-id="facing"]')
            card.wait_for(timeout=15000)
            assert 'bumi_sound_facing' in card.inner_text()
            values=card.locator('[data-key="action"] option').evaluate_all('(items)=>items.map(x=>x.value)')
            for action in ('prepare_authorization','enable_follow','authorize_follow','renew_follow','disable_follow','cancel'):assert action in values,values
            card.locator('.canvas-card-info-btn').click()
            page.wait_for_timeout(300)
            page.screenshot(path=str(args.output/'canvas.png'),full_page=True)
            (args.output/'browser.json').write_text(json.dumps(dict(card_visible=True,actions=values,details_opened=True,
                project_started=False,lifecycle_via_project_controls=True,tool_motion_executed=False,hardware_verified=False,page_errors=errors),indent=2))
        except Exception:
            page.screenshot(path=str(args.output/'failure.png'),full_page=True)
            (args.output/'failure.txt').write_text(json.dumps({'errors':errors,'body':page.locator('body').inner_text()[:12000]},ensure_ascii=False,indent=2))
            raise
        finally:browser.close()

if __name__=='__main__':main()
