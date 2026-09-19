"""Launch and monitor one fresh bounded Stage1 recording."""
from pathlib import Path
import hashlib,json,sys,time,uuid
from ops import D,source
label=sys.argv[1];ticks=int(sys.argv[2]) if len(sys.argv)>2 else 12
resolution=sys.argv[3] if len(sys.argv)>3 else 'native'
wall=int(sys.argv[4]) if len(sys.argv)>4 else 600
assert label.replace('_','').isalnum() and not (D/(label+'_launch.json')).exists()
PRE='''from pathlib import Path
import sys,json,time,os,subprocess
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');S=M/'stage1';C=S/'collection_20260918_v1'
sys.path[:0]=[str(C),str(S/'scripts'),str(M/'scripts'),str(M)]
from mro_runtime.paths import read_json,atomic_replace_json
from stage1_supervisor import identity,still_live
from mro_runtime.launcher import gpu_snapshot,_top_header
cfg=read_json(M,S/'config/viewing_session.json');run=S/'runtime/run'/cfg['consumed_by_launch_id']
'''
for _ in range(60):
    state=json.loads(source(PRE+"supervisor=read_json(M,S/'manifests/current_view_supervisor.json');print(json.dumps({'owner':read_json(M,run/'owner.json'),'stage':read_json(M,run/'stage_status.json') if (run/'stage_status.json').exists() else {},'supervisor':supervisor,'live':still_live(supervisor['identity'])}))\n"))
    current=state['owner'].get('supervisor_pid')==state['supervisor']['pid']
    if current and state['owner']['status']=='running' and state['stage'].get('status')=='stage1_open':break
    if not state['live'] or (current and state['owner']['status']=='stopped'):raise RuntimeError(state)
    time.sleep(2)
else:raise TimeoutError('Stage1 ready')
rid=uuid.uuid4().hex
spec={'schema':'stage1_recording_v1','mode':'smoke' if ticks<=60 else ('pilot' if ticks==300 else 'route'),
      'ticks':ticks,'physics_hz':60,'camera_stride':4,'resolution':resolution,'wall_timeout_s':wall,
      'color_plane_enabled':False,'qr_boards_enabled':False,'lidar_enabled':True,'epoch':int(rid[:8],16),'request_id':rid}
expected_sources={name:hashlib.sha256((D/name).read_bytes()).hexdigest() for name in (
    'record_runtime.py','record_common.py','record_render.py','record_lidar.py','record_probe.py',
    'record_worker.py','record_driver.py','timing_assessment.py','waypoints.json')}
launch=json.loads(source(PRE+'SPEC='+repr(spec)+'\nLABEL='+repr(label)+'\nEXPECTED_SOURCES='+repr(expected_sources)+'\n'+'''
import hashlib
from record_common import validate_spec
for name,expected in EXPECTED_SOURCES.items():
    path=C/name
    assert path.resolve(strict=True)==path and hashlib.sha256(path.read_bytes()).hexdigest()==expected,'Deploy this QR-free version before recording: '+name
assert not (run/'collection_consumed.json').exists()
owner=read_json(M,run/'owner.json');assert owner['status']=='running'
stage=read_json(M,run/'stage_status.json');assert not stage['recording'] and not stage['timeline_playing']
case=S/'outputs'/('collection_record_'+LABEL+'_'+SPEC['request_id'][:10]);assert case.resolve()==case;case.mkdir()
for name in ('home','tmp','logs','cache'):(case/name).mkdir()
SPEC.update(case=str(case),launch_id=cfg['consumed_by_launch_id']);validate_spec(SPEC)
atomic_replace_json(M,case/'spec.json',SPEC)
env=dict(os.environ,HOME=str(case/'home'),TMPDIR=str(case/'tmp'),XDG_CACHE_HOME=str(case/'cache'),PYTHONDONTWRITEBYTECODE='1',HISTFILE='')
with (case/'driver.stdout').open('xb') as out,(case/'driver.stderr').open('xb') as err:
    p=subprocess.Popen(['/usr/bin/python3','-B',str(C/'record_driver.py'),str(case)],cwd=case,env=env,stdin=subprocess.DEVNULL,stdout=out,stderr=err,start_new_session=True)
print(json.dumps({'case':str(case),'driver':identity(p.pid),'launch_id':SPEC['launch_id'],'request_id':SPEC['request_id'],'spec':SPEC}))
''',label+'_launch.json'));print(json.dumps(launch),flush=True)
deadline=time.monotonic()+wall+300;samples=[]
while time.monotonic()<deadline:
    time.sleep(10)
    result=json.loads(source(PRE+'CASE='+repr(launch['case'])+'\n'+'''
case=Path(CASE);result={}
for name in ('progress.json','producer_report.json','driver_report.json','publisher_done.json','subscriber_done.json','publish_error.json','subscribe_error.json'):
    if (case/name).exists():
        d=read_json(M,case/name)
        result[name]=d if name not in ('producer_report.json','driver_report.json') else {k:d[k] for k in ('status','error','traceback','qr_verification','state_count','image_count','lidar_count','resources') if k in d}
        if name=='driver_report.json' and 'resources' in result[name]:result[name]['resources']=result[name]['resources'][-1:]
result['gpu']=gpu_snapshot();result['cpu']=_top_header()
print(json.dumps(result))
'''));samples.append(result)
    (D/(label+'_monitor.json')).write_text(json.dumps(samples),encoding='utf-8')
    print(json.dumps({'progress':result.get('progress.json'),'driver':result.get('driver_report.json',{}).get('status'),
                      'producer':{k:v for k,v in result.get('producer_report.json',{}).items() if k in ('status','error','qr_verification')},
                      'gpu1':result['gpu']['devices']['1'],'cpu_idle':result['cpu']['idle_percent']}),flush=True)
    final=result.get('driver_report.json',{}).get('status')
    if final in ('complete','failed'):break
else:
    source(PRE+'case=Path('+repr(launch['case'])+");atomic_replace_json(M,case/'abort_requested.json',{'reason':'local finite deadline'})\n")
    raise TimeoutError('Recording deadline')
reports=json.loads(source(PRE+'case=Path('+repr(launch['case'])+")\nprint(json.dumps({p.name:json.loads(p.read_text()) for p in case.glob('*.json') if p.is_file()}))\n",label+'_reports.json'))
print(json.dumps({'status':final,'case':launch['case'],'error':reports.get('driver_report.json',{}).get('error')}),flush=True)
if final!='complete':raise RuntimeError(reports.get('driver_report.json',{}))
