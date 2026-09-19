"""Local transport and reviewed owned-session control for this campaign."""
from pathlib import Path
import base64
import hashlib
import importlib.util
import json
import sys
import time

D=Path(__file__).resolve().parent
W=D.parent
sp=importlib.util.spec_from_file_location('remote',W/'camera_sync_validation_20260911/remote_read.py')
remote=importlib.util.module_from_spec(sp);sp.loader.exec_module(remote)
remote.SOCKET='/home/gamja/.ssh/mro_20260918.sock'

def source(code, name=None, timeout=90):
    result=remote.run_source(code,timeout=timeout)
    if name:
        p=D/name;p.parent.mkdir(parents=True,exist_ok=True)
        with p.open('xb') as f:f.write(result)
    return result

def file_call(file,name,timeout=90):
    return source((W/file).read_text(encoding='utf-8-sig'),name,timeout)

if __name__=='__main__':
    cmd=sys.argv[1]
    if cmd=='baseline':
        raw=source('''from pathlib import Path
import json,base64,hashlib
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');S=M/'stage1'
release=json.loads((S/'manifests/release.json').read_text())
files={}
paths=[S/r for r in release]
paths += [S/'manifests/release.json',S/'manifests/deployment.json',S/'manifests/scene_build.json',S/'stages/scene.usda',S/'stages/overrides.usda',S/'config/waypoints.json',S/'config/camera_layout.json',S/'config/tracking_center.json',S/'config/camera_models.json',S/'config/viewing_session.json']
paths += [M/r for r in ('scripts/drone_lidar_adapter.py','scripts/live_lidar_packet.py','manifests/preparation_result.json','config/launch_policy.json')]
for p in dict.fromkeys(paths):
    assert p.resolve(strict=True)==p and p.is_relative_to(M)
    b=p.read_bytes();assert len(b)<5*1024**2
    files[str(p.relative_to(M))]={'data':base64.b64encode(b).decode(),'sha256':hashlib.sha256(b).hexdigest()}
for r,h in release.items():assert files['stage1/'+r]['sha256']==h,r
print(json.dumps(files))
''')
        files=json.loads(raw);out=D/'baseline';out.mkdir()
        hashes={}
        for rel,item in files.items():
            p=out/rel;p.parent.mkdir(parents=True,exist_ok=True)
            b=base64.b64decode(item['data']);assert hashlib.sha256(b).hexdigest()==item['sha256']
            with p.open('xb') as f:f.write(b)
            hashes[rel]=item['sha256']
        (D/'baseline_hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
        print(json.dumps({'baseline_files':len(files),'release_verified':True}))
    elif cmd=='stop':
        label=sys.argv[2]
        print(file_call(Path('stage1_20260914/stop_stage1.remote.py'),label+'_stop.json').decode())
    elif cmd=='start':
        label=sys.argv[2]
        print(file_call(Path('stage1_20260914/start.remote.py'),label+'_start.txt').decode())
    elif cmd=='ready':
        label=sys.argv[2]
        print(file_call(Path('stage1_20260914/wait_ready.remote.py'),label+'_ready.json').decode())
    elif cmd=='remote':
        print(source((D/sys.argv[2]).read_text(encoding='utf-8-sig'),sys.argv[3]).decode())
    else:raise ValueError(cmd)
