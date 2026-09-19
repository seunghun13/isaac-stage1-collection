"""Archive and deploy supplied Stage1 files only while the owned Kit is stopped."""
from pathlib import Path
import ast,base64,hashlib,json,sys
from ops import D,source
label=sys.argv[1]
files={}
for rel,local in {
    'collection_20260918_v1/record_lidar.py':D/'record_lidar.py',
    'collection_20260918_v1/record_probe.py':D/'record_probe.py',
    'collection_20260918_v1/record_render.py':D/'record_render.py',
    'collection_20260918_v1/record_common.py':D/'record_common.py',
    'collection_20260918_v1/record_worker.py':D/'record_worker.py',
    'collection_20260918_v1/record_runtime.py':D/'record_runtime.py',
    'collection_20260918_v1/record_driver.py':D/'record_driver.py',
    'collection_20260918_v1/waypoints.json':D/'waypoints.json',
    'scripts/view_hangar_bootstrap.py':D/'view_hangar_bootstrap.py',
}.items():
    b=local.read_bytes()
    if local.suffix=='.py':ast.parse(b.decode('utf-8'))
    else:json.loads(b)
    files[rel]={'data':base64.b64encode(b).decode(),'sha256':hashlib.sha256(b).hexdigest()}
code='FILES='+repr(files)+'\n'+'''from pathlib import Path
import sys,json,hashlib,base64,uuid
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');S=M/'stage1'
assert S.resolve(strict=True)==S
sys.path[:0]=[str(S/'scripts'),str(M/'scripts'),str(M)]
from mro_runtime.paths import read_json,atomic_replace_json
from stage1_supervisor import still_live
cfg=read_json(M,S/'config/viewing_session.json');run=S/'runtime/run'/cfg['consumed_by_launch_id']
owner=read_json(M,run/'owner.json')
assert owner['status']=='stopped' and not owner['cleanup_errors'] and not owner['remaining_owned_processes']
assert not still_live(read_json(M,S/'manifests/current_view_supervisor.json')['identity'])
release=read_json(M,S/'manifests/release.json')
for rel,h in release.items():assert hashlib.sha256((S/rel).read_bytes()).hexdigest()==h,rel
archive=S/'manifests'/('collection_revision_'+uuid.uuid4().hex)
assert archive.resolve()==archive;archive.mkdir()
with (archive/'release.json').open('xb') as f:f.write((S/'manifests/release.json').read_bytes())
for rel,item in FILES.items():
    p=S/rel;assert p.resolve()==p and p.is_relative_to(S)
    p.parent.mkdir(parents=True,exist_ok=True)
    if p.exists():
        old=archive/rel;old.parent.mkdir(parents=True,exist_ok=True)
        with old.open('xb') as f:f.write(p.read_bytes())
    b=base64.b64decode(item['data']);assert hashlib.sha256(b).hexdigest()==item['sha256']
    tmp=p.with_name(p.name+'.new');assert not tmp.exists()
    with tmp.open('xb') as f:f.write(b)
    tmp.replace(p);release[rel]=item['sha256']
atomic_replace_json(M,S/'manifests/release.json',release)
print(json.dumps({'status':'deployed_while_owned_stopped','archive':str(archive),'files':{k:v['sha256'] for k,v in FILES.items()}}))
'''
print(source(code,label+'_deployment.json').decode())
