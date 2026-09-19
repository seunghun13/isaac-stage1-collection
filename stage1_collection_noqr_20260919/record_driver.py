"""Bounded trial driver. Only this driver's explicitly owned ROS children stop."""
import json,os,sys,time,subprocess,traceback,hashlib,fcntl
from pathlib import Path
from record_common import M,S,C as V,safe,save,validate_spec,SIZES
case=safe(sys.argv[1]);spec=validate_spec(json.loads((case/'spec.json').read_text()))
sys.path[:0]=[str(S/'scripts'),str(M/'scripts'),str(M)]
from mro_runtime.paths import read_json,atomic_replace_json
from stage1_supervisor import still_live,identity
from stage1_storage_budget import check_storage_budget
from mro_runtime.launcher import gpu_snapshot,_top_header
run=S/'runtime/run'/spec['launch_id'];owner=read_json(M,run/'owner.json')
assert owner['status']=='running'
if spec['mode']=='route':
    acceptance=read_json(M,S/'manifests/recording_acceptance.json')
    assert acceptance.get('synchronized_dataset_pass') is True,'Full-route synchronization gate has not passed'
lock=safe(S/'runtime/run/collection_driver.lock').open('a+b');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
stage=read_json(M,run/'stage_status.json');assert stage['status']=='stage1_open' and not stage['timeline_playing'] and not stage['recording']
report={'status':'running','case':str(case),'spec':spec,'resources':[],'started_wall_ns':str(time.time_ns())};children=[];requested=False
expected_frames=spec['ticks']//4+1
pixels=sum(w*h*3 for w,h in SIZES.values()) if spec['resolution']=='native' else 2050000
storage_reservation=int(expected_frames*pixels*1.15)+512*1024**2
assert storage_reservation<100*1024**3

def resource():
    latest=read_json(M,run/'latest_resources.json');assert latest['classification'] is None
    assert read_json(M,run/'owner.json')['status']=='running'
    gpu=gpu_snapshot();cfg=read_json(M,run/'configuration.json');owned=latest['owned_processes'];kit=[]
    for p in gpu['processes']:
        now=identity(p['pid'])
        if p['gpu_uuid']==gpu['devices'][1]['uuid']:
            if p['pid']==cfg['xorg']['pid']:
                assert all(now.get(k)==cfg['xorg'][k] for k in ('pid','start_ticks','uid','exe'))
            else:
                assert any(x['pid']==now['pid'] and x['start_ticks']==now['start_ticks'] for x in owned)
                kit.append(now)
    assert len(kit)==1
    return {'wall_ns':str(time.time_ns()),'gpu':gpu,'top':_top_header(),'kit':kit,'storage':check_storage_budget(M,storage_reservation)}
def wait_file(name,timeout):
    end=time.monotonic()+timeout
    while not (case/name).exists():
        if any(p.poll() is not None for _,p in children):raise RuntimeError('ROS worker exited before '+name)
        if time.monotonic()>end:raise TimeoutError(name)
        time.sleep(.1)
def start(role):
    B=M/'issacsim/exts/isaacsim.ros2.bridge/humble';T=M/'runtime/tools/rosbags-0.11.5-py311'
    env={'PATH':'/usr/bin:/bin','HOME':str(case/'home'),'TMPDIR':str(case/'tmp'),'XDG_CACHE_HOME':str(case/'cache'),
         'XDG_CONFIG_HOME':str(case/'home'),'XDG_DATA_HOME':str(case/'home'),'ROS_LOG_DIR':str(case/'logs'),'ROS_HOME':str(case/'home'),
         'PYTHONDONTWRITEBYTECODE':'1','PYTHONNOUSERSITE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1',
         'PYTHONPATH':str(T/'site')+':'+str(B/'rclpy'),'LD_LIBRARY_PATH':str(B/'lib')+':'+str(M/'issacsim/kit/python/lib'),
         'ROS_DISTRO':'humble','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_DOMAIN_ID':'82','ROS_LOCALHOST_ONLY':'0',
         'FASTRTPS_DEFAULT_PROFILES_FILE':str(S/'validation_20260918_v1/fastdds_local_udp.xml'),'FASTDDS_DEFAULT_PROFILES_FILE':str(S/'validation_20260918_v1/fastdds_local_udp.xml'),
         'RMW_FASTRTPS_USE_QOS_FROM_XML':'1','RMW_FASTRTPS_PUBLICATION_MODE':'SYNCHRONOUS','CUDA_VISIBLE_DEVICES':'','HISTFILE':'',
         'RCUTILS_LOGGING_USE_STDOUT':'1','LANG':'C.UTF-8','LC_ALL':'C.UTF-8'}
    args=['/usr/bin/bwrap','--ro-bind','/','/','--ro-bind','/dev/shm','/dev/shm','--bind',str(case),str(case),
          '--proc','/proc','--unshare-pid','--die-with-parent','--new-session','--chdir',str(case),'--',
          str(M/'issacsim/kit/python/bin/python3'),'-B',str(V/'record_worker.py'),role,str(case)]
    with (case/(role+'.stdout')).open('xb') as out,(case/(role+'.stderr')).open('xb') as err:
        p=subprocess.Popen(args,env=env,cwd=case,stdin=subprocess.DEVNULL,stdout=out,stderr=err)
    children.append((role,p))
try:
    for rel in ('manifests/preparation_result.json','config/launch_policy.json'):read_json(M,rel)
    report['resources'].append(resource());report['source_hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in V.iterdir() if p.is_file()}
    save(case/'driver_report.json',report)
    start('subscribe');wait_file('subscriber_ready.json',40);start('publish');wait_file('publisher_ready.json',50)
    req={'action':'stage1_record','case':str(case),'request_id':spec['request_id'],'expected_launch_id':spec['launch_id']}
    previous=run/'request.json'
    if previous.exists():
        prior=read_json(M,previous);response=read_json(M,run/'response.json');assert response['request_id']==prior['request_id'] and response['status'] in ('complete','failed')
    atomic_replace_json(M,previous,req);requested=True;deadline=time.monotonic()+spec['wall_timeout_s'];nextresource=0
    while time.monotonic()<deadline:
        if (run/'response.json').exists():
            response=read_json(M,run/'response.json')
            if response.get('request_id')==spec['request_id']:
                report['response']=response
                if response['status']!='complete':raise RuntimeError('Runtime failed: '+str(response))
                break
        if any(p.poll() not in (None,0) for _,p in children):raise RuntimeError('ROS worker failed')
        if time.monotonic()>=nextresource:
            report['resources'].append(resource());save(case/'driver_report.json',report);nextresource=time.monotonic()+10
        time.sleep(.25)
    else:raise TimeoutError('Trial finite bound')
    deadline=time.monotonic()+200
    while any(p.poll() is None for _,p in children):
        if time.monotonic()>deadline:raise TimeoutError('ROS terminal drain')
        time.sleep(.2)
    assert all(p.returncode==0 for _,p in children)
    pub=read_json(M,case/'publisher_done.json');sub=read_json(M,case/'subscriber_done.json')
    assert pub['status']==sub['status']=='complete' and pub['counts']==sub['counts']
    report.update(status='complete',published_counts=pub['counts'],received_counts=sub['counts'])
except BaseException as exc:
    report.update(status='failed',error=repr(exc),traceback=traceback.format_exc())
    if requested:
        save(case/'abort_requested.json',{'reason':repr(exc)})
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            if (run/'response.json').exists() and read_json(M,run/'response.json').get('request_id')==spec['request_id']:break
            time.sleep(.2)
finally:
    for label,p in children:
        if p.poll() is None:
            p.terminate()
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=8)
    report['children']=[{'role':label,'pid':p.pid,'returncode':p.poll()} for label,p in children]
    try:
        report['cooldown_resources']=[]
        for _ in range(3):
            time.sleep(2);report['cooldown_resources'].append(resource())
        deployment=read_json(M,S/'manifests/deployment.json')
        report['original_sources_preserved']={rel:hashlib.sha256((M/rel).read_bytes()).hexdigest()==h for rel,h in deployment['protected_sources'].items()}
        assert all(report['original_sources_preserved'].values())
    except Exception as exc:report['final_check_error']=repr(exc);report['status']='failed'
    report['finished_wall_ns']=str(time.time_ns());save(case/'driver_report.json',report)
print(json.dumps({'status':report['status'],'case':str(case),'error':report.get('error')}),flush=True)
