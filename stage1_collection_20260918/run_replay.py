from pathlib import Path
import base64,json,sys
from ops import D,source
label=sys.argv[1];case=json.loads((D/(label+'_launch.json')).read_text())['case']
code='CASE='+repr(case)+'\nDATA='+repr(base64.b64encode((D/'replay_worker.py').read_bytes()).decode())+'\n'+'''from pathlib import Path
import base64,os,subprocess,json,time
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');S=M/'stage1';case=Path(CASE)
assert case.resolve()==case and case.is_relative_to(S/'outputs')
assert json.loads((case/'driver_report.json').read_text())['status']=='complete'
out=case/'replay_v2';assert out.resolve()==out;out.mkdir()
for name in ('home','tmp','cache','logs'):(out/name).mkdir()
p=out/'replay_worker.py';p.write_bytes(base64.b64decode(DATA))
B=M/'issacsim/exts/isaacsim.ros2.bridge/humble';V=S/'validation_20260918_v1'
env={'PATH':'/usr/bin:/bin','HOME':str(out/'home'),'TMPDIR':str(out/'tmp'),'XDG_CACHE_HOME':str(out/'cache'),
 'XDG_CONFIG_HOME':str(out/'home'),'XDG_DATA_HOME':str(out/'home'),'ROS_LOG_DIR':str(out/'logs'),'ROS_HOME':str(out/'home'),
 'PYTHONDONTWRITEBYTECODE':'1','PYTHONNOUSERSITE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1',
 'PYTHONPATH':str(M/'runtime/tools/rosbags-0.11.5-py311/site')+':'+str(B/'rclpy')+':'+str(S/'collection_20260918_v1'),'LD_LIBRARY_PATH':str(B/'lib')+':'+str(M/'issacsim/kit/python/lib'),
 'ROS_DISTRO':'humble','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'0',
 'FASTRTPS_DEFAULT_PROFILES_FILE':str(V/'fastdds_local_udp.xml'),'FASTDDS_DEFAULT_PROFILES_FILE':str(V/'fastdds_local_udp.xml'),
 'RMW_FASTRTPS_USE_QOS_FROM_XML':'1','RMW_FASTRTPS_PUBLICATION_MODE':'SYNCHRONOUS','CUDA_VISIBLE_DEVICES':'','HISTFILE':'',
 'LANG':'C.UTF-8','LC_ALL':'C.UTF-8'}
children=[]
def start(role):
 with (out/(role+'.stdout')).open('xb') as stdout,(out/(role+'.stderr')).open('xb') as stderr:
  child=subprocess.Popen(['/usr/bin/bwrap','--ro-bind','/','/','--ro-bind','/dev/shm','/dev/shm','--bind',str(out),str(out),'--proc','/proc','--unshare-pid','--die-with-parent','--new-session','--chdir',str(out),'--',str(M/'issacsim/kit/python/bin/python3'),'-B',str(p),role,str(case),str(out)],cwd=out,env=env,stdin=subprocess.DEVNULL,stdout=stdout,stderr=stderr)
 children.append((role,child))
try:
 start('publish');deadline=time.monotonic()+60
 while not (out/'static_published.json').exists():
  assert time.monotonic()<deadline and children[0][1].poll() is None,'static replay startup';time.sleep(.1)
 start('subscribe')
 for role,child in children:assert child.wait(timeout=200)==0,(out/(role+'.stderr')).read_text()[-3000:]
 result={role:json.loads((out/(role+'_done.json')).read_text()) for role,_ in children}
 print(json.dumps(result))
finally:
 for role,child in children:
  if child.poll() is None:
   child.terminate()
   try:child.wait(timeout=10)
   except subprocess.TimeoutExpired:child.kill();child.wait(timeout=10)
'''
r=json.loads(source(code,label+'_replay.json',timeout=450))
print(json.dumps({'publisher_status':r['publish']['status'],'subscriber_status':r['subscribe']['status'],
                  'exact_cdr_pass':r['subscribe'].get('exact_cdr_pass'),'late_static_pass':r['subscribe'].get('late_static_pass'),
                  'received_counts':{k:len(v) for k,v in r['subscribe'].get('received_sha256',{}).items()}}))
