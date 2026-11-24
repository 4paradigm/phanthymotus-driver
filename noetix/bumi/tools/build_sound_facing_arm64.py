#!/usr/bin/env python3
"""Task-owned ARM64 build and safe smoke; never pushes or runs production CMD.

Requires an already available Docker/Buildx ARM64-capable engine. Does not start
Docker, switch contexts, install binfmt, or change a shared builder.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from uuid import uuid4

BUMI=Path(__file__).resolve().parents[1]
REPO=BUMI.parents[1]


def run(command,log,**kwargs):
    with log.open('a') as stream:
        stream.write(json.dumps(command)+'\n');stream.flush()
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=False,**kwargs)
    if result.returncode:raise RuntimeError(f'Command failed ({result.returncode}); inspect {log}')


def prepare(output):
    output.mkdir(parents=True,exist_ok=False)
    context=output/'context'
    ignore=shutil.ignore_patterns('.git','__pycache__','.pytest_cache','*.pyc','tests','tools','build','*.egg-info')
    # The SDK's source is required; only generated build outputs are omitted.
    shutil.copytree(BUMI,context,ignore=ignore)
    shutil.copytree(REPO/'common',context/'common',ignore=ignore)
    files=sorted(p for p in context.rglob('*') if p.is_file())
    manifest={str(p.relative_to(context)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (output/'SOURCE-SHA256.json').write_text(json.dumps(manifest,indent=2))
    receipt=dict(platform='linux/arm64',context=str(context),files=len(files),built=False,smoke_passed=False,pushed=False,hardware_motion=False)
    (output/'BUILD.json').write_text(json.dumps(receipt,indent=2))
    return context,receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New task-owned directory')
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--docker',default='docker')
    args=parser.parse_args()
    output=args.output.resolve()
    if output.exists():parser.error('output already exists; refusing overwrite')
    context,receipt=prepare(output)
    if args.prepare_only:
        print(json.dumps(receipt,indent=2));return 0
    docker=args.docker
    builder='bumi-sound-review-'+uuid4().hex[:12]
    image='bumi-sound-facing-local:'+uuid4().hex[:12]
    # No inherited registry credentials or default-builder changes. DOCKER_HOST
    # can select the already-authorized engine; no context is created/switched.
    config=output/'docker-config';config.mkdir()
    (config/'config.json').write_text('{}\n')
    env=os.environ.copy();env['DOCKER_CONFIG']=str(config)
    created=False
    try:
        run([docker,'version'],output/'environment.log',env=env,timeout=20)
        run([docker,'buildx','version'],output/'environment.log',env=env,timeout=20)
        run([docker,'buildx','create','--name',builder,'--driver','docker-container',
             '--driver-opt','memory=3g','--driver-opt','cpu-period=100000','--driver-opt','cpu-quota=200000'],output/'build.log',env=env,timeout=60)
        created=True
        run([docker,'buildx','build','--builder',builder,'--platform','linux/arm64','--load',
             '--tag',image,'--file',str(context/'Dockerfile'),'--progress','plain',str(context)],output/'build.log',env=env,timeout=7200)
        inspected=subprocess.run([docker,'image','inspect',image],capture_output=True,text=True,check=True,env=env,timeout=20)
        image_info=json.loads(inspected.stdout)[0]
        if image_info['Architecture']!='arm64' or image_info['Os']!='linux':raise RuntimeError('Image architecture mismatch')
        receipt.update(built=True,image=image,image_id=image_info['Id'],bytes=image_info['Size'])
        (output/'BUILD.json').write_text(json.dumps(receipt,indent=2))
        smoke=BUMI/'tools/smoke_sound_facing_image.py'
        run([docker,'run','--rm','--name','bumi-sound-smoke-'+uuid4().hex[:12],
             '--platform','linux/arm64','--network','none','--read-only','--cap-drop','ALL',
             '--security-opt','no-new-privileges','--user','65534:65534','--memory','512m','--cpus','1',
             '--tmpfs','/tmp:rw,nosuid,nodev,size=32m','--env','PYTHONDONTWRITEBYTECODE=1',
             '--mount',f'type=bind,src={smoke},dst=/tmp/smoke.py,readonly',
             '--entrypoint','/bin/bash',image,'-c','source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /bumi_ros_ws/install/setup.bash && python3 /tmp/smoke.py'],output/'smoke.log',env=env,timeout=90)
        receipt['smoke_passed']=True
    except (OSError,RuntimeError,subprocess.SubprocessError) as exc:
        receipt['blocker']=str(exc)
        print(str(exc),file=sys.stderr)
    finally:
        if created:
            # Exact task-created name only; never prune or remove an image.
            try:
                run([docker,'buildx','rm',builder],output/'cleanup.log',env=env,timeout=60)
            except (OSError,RuntimeError,subprocess.SubprocessError) as exc:
                receipt['cleanup_error']=str(exc)
        (output/'BUILD.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(receipt,indent=2))
    return 0 if receipt.get('smoke_passed') and not receipt.get('cleanup_error') else 1

if __name__=='__main__':raise SystemExit(main())
