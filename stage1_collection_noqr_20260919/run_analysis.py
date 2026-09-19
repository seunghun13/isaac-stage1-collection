from pathlib import Path
import base64,json,sys
from ops import D,source
label=sys.argv[1];case=json.loads((D/(label+'_launch.json')).read_text())['case']
payload=base64.b64encode((D/'analyze_bag.py').read_bytes()).decode()
code='CASE='+repr(case)+'\nDATA='+repr(payload)+'\n'+'''from pathlib import Path
import base64,os,subprocess,json
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');S=M/'stage1';case=Path(CASE)
assert case.resolve()==case and case.is_relative_to(S/'outputs')
p=case/'analyze_bag_v2.py';assert not p.exists();p.write_bytes(base64.b64decode(DATA))
env=dict(os.environ,HOME=str(case/'home'),TMPDIR=str(case/'tmp'),PYTHONDONTWRITEBYTECODE='1',
  PYTHONPATH=':'.join(str(x) for x in (M/'issacsim/exts/omni.pip.compute/pip_prebundle',M/'runtime/tools/rosbags-0.11.5-py311/site',S/'validation_20260918_v1',S/'collection_20260918_v1',M/'scripts')),
  OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',CUDA_VISIBLE_DEVICES='')
with (case/'analysis_v2.stdout').open('xb') as out,(case/'analysis_v2.stderr').open('xb') as err:
 child=subprocess.Popen(['/usr/bin/bwrap','--ro-bind','/','/','--bind',str(case),str(case),'--proc','/proc','--unshare-pid','--die-with-parent','--new-session','--chdir',str(case),'--',str(M/'issacsim/kit/python/bin/python3'),'-B',str(p),str(case)],cwd=case,env=env,stdin=subprocess.DEVNULL,stdout=out,stderr=err)
 rc=child.wait(timeout=900)
assert rc==0,(case/'analysis_v2.stderr').read_text()[-6000:]
print((case/'analysis.json').read_text())
'''
raw=source(code,label+'_analysis.json',timeout=950)
r=json.loads(raw)
print(json.dumps({k:v for k,v in r.items() if k not in ('cameras','topics')}))
print(json.dumps({cid:{k:v for k,v in x.items() if k!='image_rows'} for cid,x in r['cameras'].items()}))
