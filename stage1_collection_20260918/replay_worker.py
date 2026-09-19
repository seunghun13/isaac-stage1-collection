"""Finite stored-CDR DDS replay, including late static subscribers. No bag writes."""
import collections,hashlib,json,os,sqlite3,sys,time,traceback
from pathlib import Path
from record_common import safe,save,topics
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import PoseStamped,Vector3Stamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

role=sys.argv[1];case=safe(sys.argv[2]);out=safe(sys.argv[3])
classes={c.__name__:c for c in (Image,CameraInfo,PointCloud2,PoseStamped,Vector3Stamped,TFMessage,Clock,String)}
specs=topics();static=('session','waypoints','static')
def qos(key):return QoSProfile(depth=2 if key.endswith('_image') else 128,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL if key in static else DurabilityPolicy.VOLATILE)
dbpath=next((case/'bag').glob('*.db3'));db=sqlite3.connect('file:'+str(dbpath)+'?mode=ro&immutable=1',uri=True)
ids={name:i for i,name in db.execute('SELECT id,name FROM topics')}
selected={}
for key,(topic,typ) in specs.items():
    rows=db.execute('SELECT id FROM messages WHERE topic_id=? ORDER BY id',(ids[topic],)).fetchall()
    assert rows,topic
    selected[key]=list(dict.fromkeys([rows[0][0]] if key in static else [rows[0][0],rows[-1][0]]))
expected={key:[hashlib.sha256(db.execute('SELECT data FROM messages WHERE id=?',(i,)).fetchone()[0]).hexdigest() for i in rows] for key,rows in selected.items()}
rclpy.init(args=[]);node=rclpy.create_node('stage1_replay_'+role)
report={'role':role,'status':'running','started_wall_ns':str(time.time_ns()),'expected_sha256':expected}
try:
    if role=='publish':
        pubs={key:node.create_publisher(classes[typ.split('/')[-1]],topic,qos(key)) for key,(topic,typ) in specs.items()}
        def send(key):
            for i in selected[key]:pubs[key].publish(bytes(db.execute('SELECT data FROM messages WHERE id=?',(i,)).fetchone()[0]))
        for key in static:send(key)
        save(out/'static_published.json',{'wall_ns':str(time.time_ns()),'keys':static,'subscriber_started_yet':False})
        deadline=time.monotonic()+60
        while not all(p.get_subscription_count() for p in pubs.values()):
            assert time.monotonic()<deadline,'replay discovery';rclpy.spin_once(node,timeout_sec=.05)
        for key in specs:
            if key not in static:send(key)
        save(out/'publisher_sent.json',{'wall_ns':str(time.time_ns()),'counts':{k:len(v) for k,v in selected.items()}})
        deadline=time.monotonic()+150
        while not (out/'subscribe_done.json').exists():
            assert time.monotonic()<deadline,'replay drain';rclpy.spin_once(node,timeout_sec=.05)
        report['status']='complete'
    else:
        report['static_already_published']=json.loads((out/'static_published.json').read_text())
        received={k:[] for k in specs}
        def callback(key):
            def got(raw):received[key].append(hashlib.sha256(raw).hexdigest())
            return got
        subscriptions=[node.create_subscription(classes[typ.split('/')[-1]],topic,callback(key),qos(key),raw=True) for key,(topic,typ) in specs.items()]
        deadline=time.monotonic()+150
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if all(collections.Counter(received[k])==collections.Counter(expected[k]) for k in expected):break
        report['received_sha256']=received
        report['exact_cdr_pass']=all(collections.Counter(received[k])==collections.Counter(expected[k]) for k in expected)
        report['late_static_pass']=all(received[k]==expected[k] for k in static)
        report['status']='complete' if report['exact_cdr_pass'] and report['late_static_pass'] else 'failed'
except BaseException as exc:report.update(status='failed',error=repr(exc),traceback=traceback.format_exc());raise
finally:
    report['finished_wall_ns']=str(time.time_ns());save(out/(role+'_done.json'),report)
    node.destroy_node();rclpy.shutdown();db.close()
