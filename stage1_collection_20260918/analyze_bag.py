"""Read-only bag verification independent of producer QR decisions."""
import collections,hashlib,json,math,sqlite3,sys,time
from pathlib import Path
import numpy as np
from rosbags.typesys import Stores,get_typestore
from marker_codec import decode_marker
from board_vision import detect_board

case=Path(sys.argv[1]);root=Path('/mnt/DATA/workspace/ws_minho/mro_1/stage1')
assert case.resolve()==case and case.is_relative_to(root/'outputs')
spec=json.loads((case/'spec.json').read_text());producer=json.loads((case/'producer_report.json').read_text())
paths=list((case/'bag').glob('*.db3'));assert len(paths)==1
dbpath=paths[0];before=dbpath.stat();db=sqlite3.connect('file:'+str(dbpath)+'?mode=ro&immutable=1',uri=True)
integrity=db.execute('PRAGMA quick_check').fetchall();assert integrity==[('ok',)]
topics={i:(name,typ) for i,name,typ in db.execute('SELECT id,name,type FROM topics')}
counts={name:db.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?',(i,)).fetchone()[0] for i,(name,typ) in topics.items()}
store=get_typestore(Stores.ROS2_HUMBLE)
def messages(topic):
    i=next(i for i,(name,typ) in topics.items() if name==topic);typ=topics[i][1]
    for stamp,raw in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY id',(i,)):
        yield stamp,store.deserialize_cdr(raw,typ)
def ns(header):return int(header.stamp.sec)*10**9+int(header.stamp.nanosec)
mapping=[json.loads(msg.data) for _,msg in messages('/stage1/time_mapping')]
states={h['index']:h for h in mapping if h['type']=='state'}
images={(h['camera_id'],h['stamp_ns']):h for h in mapping if h['type']=='image'}
lidars=[h for h in mapping if h['type']=='lidar']
lookup={h['clock']['step']:h['stamp_ns'] for h in states.values()}
session=json.loads(next(messages('/stage1/session'))[1].data)
pub=json.loads((case/'publisher_done.json').read_text());sub=json.loads((case/'subscriber_done.json').read_text())
result={'case':str(case),'sqlite_integrity':'ok','bytes':sum(p.stat().st_size for p in (case/'bag').iterdir() if p.is_file()),
        'topics':counts,'clock_checks':{},'cameras':{},'lidar':{},'errors':[],
        'lidar_acquisition_time_verified':False,'replay_ros2_cli_verified':False}
from record_common import topics as specs
expected={topic:pub['counts'][key] for key,(topic,typ) in specs().items()}
result['counts_match_publisher_dds_bag']=pub['status']==sub['status']=='complete' and pub['counts']==sub['counts'] and counts==expected
assert len(states)==spec['ticks']+1 and list(states)==list(range(spec['ticks']+1))
result['clock_checks']['strictly_increasing']=all(states[i]['stamp_ns']>states[i-1]['stamp_ns'] for i in range(1,len(states)))
result['clock_checks']['observed_elapsed_s']=states[spec['ticks']]['clock']['global_s']-states[0]['clock']['global_s']
result['clock_checks']['header_global_max_error_ns']=max(abs(h['stamp_ns']-round(h['clock']['global_s']*1e9)) for h in states.values())
for topic,kind in [('/stage1/drone/gt_pose','pose'),('/stage1/drone/gt_velocity','velocity'),('/clock','clock')]:
    for index,(_,msg) in enumerate(messages(topic)):
        stamp=msg.clock.sec*10**9+msg.clock.nanosec if kind=='clock' else ns(msg.header)
        assert stamp==states[index]['stamp_ns']
        if kind=='pose':assert np.max(np.abs(np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])-states[index]['gt_center']['position']))<1e-12
        if kind=='velocity':assert np.max(np.abs(np.array([msg.vector.x,msg.vector.y,msg.vector.z])-states[index]['velocity_world_m_s']))<1e-12
for cid in ('cam_01','cam_02','cam_03'):
    infos={ns(m.header):m for _,m in messages('/stage1/cameras/'+cid+'/camera_info')}
    cal=next(c for c in producer['cameras'] if c['camera_id']==cid)
    report={'images':0,'reference_valid':0,'reference_unreadable':0,'reference_mismatch':0,
            'drone_valid':0,'drone_unreadable':0,'drone_mismatch':0,'image_rows':[],
            'reference_global_max_error_ns':0,'native_annotator_global_max_error_ns':0,'source_hash_checks':0}
    previous=-1
    for _,msg in messages('/stage1/cameras/'+cid+'/image_raw'):
        stamp=ns(msg.header);assert stamp>previous;previous=stamp
        meta=images[(cid,stamp)];info=infos[stamp]
        assert msg.width==cal['width'] and msg.height==cal['height'] and msg.encoding=='rgb8' and msg.step==msg.width*3
        assert info.width==msg.width and info.height==msg.height and info.header.frame_id==msg.header.frame_id==cid+'/optical'
        assert np.array_equal(info.k,cal['k'])
        pixels=np.asarray(msg.data,dtype=np.uint8).reshape(msg.height,msg.width,3)
        if meta.get('source_sha256'):
            assert hashlib.sha256(pixels).hexdigest()==meta['source_sha256'];report['source_hash_checks']+=1
        decoded=decode_marker(pixels,producer['scene_markers']['cameras'][cid]['layout'])
        match=decoded['valid'] and decoded['epoch']==spec['epoch'] and decoded['camera_id']==int(cid[-2:]) and lookup.get(decoded['tick'])==stamp
        report['reference_valid' if decoded['valid'] else 'reference_unreadable']+=1
        report['reference_mismatch']+=int(decoded['valid'] and not match)
        row={'index':meta['index'],'stamp_ns':stamp,'reference':decoded,'reference_match':bool(match)}
        if producer['drone_board']['enabled']:
            board=detect_board(pixels,[0,0,msg.width,msg.height],producer['drone_board'])
            code=board.get('decoded',board.get('code',{}))
            # Current independent decoder returns its payload in `decode`.
            if not code:code=board.get('decode',{})
            valid=bool(board.get('valid'));good=valid and code.get('epoch')==spec['epoch'] and lookup.get(code.get('tick'))==stamp
            report['drone_valid' if valid else 'drone_unreadable']+=1;report['drone_mismatch']+=int(valid and not good)
            row['drone']={'valid':valid,'status':board.get('status'),'code':code,'match':bool(good)}
        report['reference_global_max_error_ns']=max(report['reference_global_max_error_ns'],abs(round(meta['reference_sim_s']*1e9)-stamp))
        report['native_annotator_global_max_error_ns']=max(report['native_annotator_global_max_error_ns'],abs(round(meta['native_simulation_time']['simulationTime']*1e9)-stamp))
        report['images']+=1;report['image_rows'].append(row)
    assert report['images']==spec['ticks']//4+1 and len(infos)==report['images']
    result['cameras'][cid]=report
lidar_report={'packets':0,'points':0,'outside_native_frame_points':0,'native_frame_deltas':[],
              'reference_global_max_error_ns':0,'native_minus_global_observation_ns':[],
              'point_binary_time_formula_verified':True,'point_payload_hash_verified':True,
              'full_rotary_scan_assembled':False,'deskew_applied':False}
from live_lidar_packet import RECORD_DTYPE
previous=None
for sequence,(_,msg) in enumerate(messages('/stage1/drone/lidar/points_raw')):
    h=lidars[sequence];meta=h['native'];assert ns(msg.header)==h['native_header_ns']
    assert msg.header.frame_id=='drone_1/lidar' and msg.point_step==32 and msg.width==h['pointcloud']['width'] and len(msg.data)==msg.row_step==32*msg.width
    body=memoryview(msg.data);assert hashlib.sha256(body).hexdigest()==h['pointcloud']['data_sha256']
    records=np.frombuffer(body,dtype=RECORD_DTYPE)
    times=records['native_time_ns_lo'].astype(np.uint64)|(records['native_time_ns_hi'].astype(np.uint64)<<np.uint64(32))
    formula=np.array([h['native_header_ns']+int(x) for x in records['native_time_offset_ns']],dtype=np.uint64)
    assert np.array_equal(times,formula)
    start=int(meta['frameStart']['timestamp_ns']);end=int(meta['frameEnd']['timestamp_ns'])
    outside=int(np.count_nonzero((times<start)|(times>end)));assert outside==h['pointcloud']['valid_points_outside_native_frame']
    lidar_report['outside_native_frame_points']+=outside;lidar_report['points']+=msg.width;lidar_report['packets']+=1
    lidar_report['reference_global_max_error_ns']=max(lidar_report['reference_global_max_error_ns'],abs(round(meta['reference_sim_s']*1e9)-h['stamp_ns']))
    lidar_report['native_minus_global_observation_ns'].append(h['native_header_ns']-h['stamp_ns'])
    if previous is not None:lidar_report['native_frame_deltas'].append(int(meta['frame_id'])-previous)
    previous=int(meta['frame_id'])
assert len(lidars)==lidar_report['packets']
lidar_report['native_frame_delta_histogram']=dict(collections.Counter(lidar_report.pop('native_frame_deltas')))
offsets=lidar_report.pop('native_minus_global_observation_ns')
lidar_report['native_minus_global_observation_range_ns']=[min(offsets),max(offsets)] if offsets else None
result['lidar']=lidar_report
result['static_tf_count']=len(next(messages('/tf_static'))[1].transforms);assert result['static_tf_count']==5
result['camera_time_pass']=all(c['reference_valid']>0 and c['reference_mismatch']==0 and c['drone_mismatch']==0 and c['reference_global_max_error_ns']<=1000 and c['native_annotator_global_max_error_ns']<=1000 for c in result['cameras'].values())
result['transport_pass']=result['counts_match_publisher_dds_bag'] and not producer['cleanup_errors']
result['synchronized_dataset_pass']=result['camera_time_pass'] and result['transport_pass'] and result['lidar_acquisition_time_verified']
after=dbpath.stat();assert before.st_size==after.st_size and before.st_mtime_ns==after.st_mtime_ns
result['read_only_bag_unchanged_stat']=True
hashobj=hashlib.sha256()
with dbpath.open('rb') as f:
    for block in iter(lambda:f.read(8*1024**2),b''):hashobj.update(block)
result['bag_sha256']=hashobj.hexdigest();result['analyzed_wall_ns']=str(time.time_ns())
out=case/'analysis.json';assert not out.exists();out.write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('cameras','topics')}))
