"""Separate Stage1 viewer: one timeline-driven drone, three static cameras."""
import asyncio,datetime,json,os,sys,time,traceback,uuid,hashlib,math
from pathlib import Path
import carb
import omni.kit.app,omni.timeline,omni.usd
M=Path('/mnt/DATA/workspace/ws_minho/mro_1').resolve(strict=True)
S=M/'stage1';assert S.resolve(strict=True)==S
RUN=Path(os.environ['MRO_VIEW_RUN']).resolve(strict=True)
assert RUN.is_relative_to(S/'runtime/run')
sys.path[:0]=[str(S/'scripts'),str(M/'scripts'),str(M)]
import stage1_scene as scene
import waypoint_motion as motion
from mro_runtime.paths import read_json,atomic_replace_json

def status(**values):
    values.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),launch_id=os.environ['MRO_LAUNCH_ID'])
    atomic_replace_json(M,RUN/'stage_status.json',values)

def render_diagnostics(stage,viewport,settings):
    from pxr import UsdGeom,Usd
    product=stage.GetPrimAtPath(viewport.render_product_path)
    return {'settings':{k:settings.get(k) for k in ('/rtx/rendermode','/persistent/rtx/rendermode','/persistent/rtx/modes/rt/enabled','/persistent/rtx/modes/pt/enabled','/persistent/rtx/modes/rt2/enabled','/rtx/post/aa/op','/rtx/post/dlss/execMode','/rtx/ecoMode/enabled')},
        'viewport':{'camera':str(viewport.camera_path),'resolution':list(viewport.resolution),'updates_enabled':viewport.updates_enabled,'render_product_path':str(viewport.render_product_path)},
        'render_product_camera':[str(p) for p in product.GetRelationship('camera').GetTargets()] if product else None,
        'render_product_attributes':{a.GetName():str(a.Get()) for a in product.GetAttributes()} if product else None,
        'mesh_count':sum(1 for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)),
        'lights':[{'path':str(p.GetPath()),'type':p.GetTypeName(),'intensity':str(p.GetAttribute('inputs:intensity').Get()),'visibility':UsdGeom.Imageable(p).ComputeVisibility()} for p in stage.Traverse() if 'Light' in p.GetTypeName()],
        'world_visibility':UsdGeom.Imageable(stage.GetPrimAtPath('/World')).ComputeVisibility()}

async def main():
    try:
        app=omni.kit.app.get_app();timeline=omni.timeline.get_timeline_interface();timeline.stop()
        settings=carb.settings.get_settings()
        for key,value in {'/app/file/ignoreUnsavedOnExit':True,'/app/livestream/allowResize':False,
            '/app/runLoops/main/rateLimitEnabled':True,'/app/runLoops/main/rateLimitFrequency':10,
            '/rtx/rendermode':'RaytracedLighting','/rtx/post/aa/op':3,'/rtx/post/dlss/execMode':2,
            '/omni/replicator/captureOnPlay':False,'/persistent/omni/replicator/captureOnPlay':False}.items():settings.set(key,value)
        status(status='opening_stage1',timeline_playing=False,recording=False)
        for _ in range(20):await app.next_update_async()
        context=omni.usd.get_context();result=await context.open_stage_async(str(scene.path('stages/scene.usda')))
        if isinstance(result,tuple) and not result[0]:raise RuntimeError(str(result))
        stage=context.get_stage();assert stage and stage.GetPrimAtPath('/World/Aircraft')
        created_now=not scene.path('manifests/scene_build.json').exists()
        report=scene.build(stage)
        if created_now:
            # Reopen the persisted composition so Hydra receives a fresh stage.
            result=await context.open_stage_async(str(scene.path('stages/scene.usda')))
            if isinstance(result,tuple) and not result[0]:raise RuntimeError(str(result))
            stage=context.get_stage();assert stage and stage.GetPrimAtPath('/World/Aircraft')
            scene.build(stage)  # Verify the persisted scene and input hashes.
        stage.GetRootLayer().SetPermissionToSave(False);stage.SetEditTarget(stage.GetSessionLayer())
        from pxr import Gf,UsdGeom
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        viewport=None
        for _ in range(100):
            viewport=get_active_viewport()
            if viewport:break
            await app.next_update_async()
        assert viewport
        camera=UsdGeom.Camera.Define(stage,'/Stage1_Overview')
        camera.CreateFocalLengthAttr(12.);camera.CreateHorizontalApertureAttr(36.);camera.CreateVerticalApertureAttr(20.25)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1,1000.));camera.CreateFStopAttr(0.)
        camera.AddTransformOp().Set(Gf.Matrix4d(1.).SetLookAt(Gf.Vec3d(30,-16,15),Gf.Vec3d(5,20,1.5),Gf.Vec3d(0,0,1)).GetInverse())
        viewport.camera_path='/Stage1_Overview';viewport.resolution=(1920,1080)
        viewport.updates_enabled=True
        timeline.set_current_time(0.);timeline.pause();timeline.set_looping(False)
        trajectory=motion.prepare(json.loads(scene.path('config/waypoints.json').read_text()))
        last_request=None;previous=0.;session_id=uuid.uuid4().hex
        scene.write_json('manifests/loaded_scene.json',{'launch_id':os.environ['MRO_LAUNCH_ID'],'session_id':session_id,
            'scene':str(scene.path('stages/scene.usda')),'readback':scene.inspect(stage,0.),'recorder_active':False})
        while app.is_running():
            await app.next_update_async()
            request_path=RUN/'request.json'
            if request_path.exists():
                req=read_json(M,request_path);rid=req.get('request_id')
                if rid and rid!=last_request:
                    last_request=rid
                    try:
                        assert req.get('expected_launch_id')==os.environ['MRO_LAUNCH_ID']
                        action=req.get('action');answer={}
                        if action=='inspect':answer={**scene.inspect(stage,timeline.get_current_time()),'render_diagnostics':render_diagnostics(stage,viewport,settings)}
                        elif action in ('seek','reset'):
                            t=0. if action=='reset' else float(req['simulation_time_s'])
                            assert math.isfinite(t) and 0<=t<=trajectory['arrival_times_s'][-1]
                            timeline.pause();timeline.set_current_time(t)
                            for _ in range(3):await app.next_update_async()
                            answer={'observed':scene.inspect(stage,t),'expected':motion.sample(trajectory,t),'timeline_s':timeline.get_current_time()}
                        elif action=='capture_views':
                            assert not timeline.is_playing()
                            directory=scene.path('outputs/preview_'+uuid.uuid4().hex);directory.mkdir(mode=0o700)
                            views=[('overview','/Stage1_Overview',(1920,1080))]+[(c,'/MRO_Cameras/'+c,(1064,920) if c=='cam_02' else (1920,1080)) for c in ('cam_01','cam_02','cam_03')]
                            images=[]
                            try:
                                for name,cam,resolution in views:
                                    viewport.camera_path=cam;viewport.resolution=resolution
                                    await asyncio.wait_for(viewport.wait_for_rendered_frames(16),timeout=50)
                                    output=directory/(name+'.png')
                                    delegate=capture_viewport_to_file(viewport,str(output),is_hdr=False)
                                    await asyncio.wait_for(delegate.wait_for_result(),timeout=50)
                                    for _ in range(100):
                                        data=output.read_bytes() if output.exists() else b''
                                        if data.endswith(b'\x00\x00\x00\x00IEND\xaeB`\x82'):break
                                        await asyncio.sleep(.1)
                                    else:raise RuntimeError('Preview PNG incomplete')
                                    images.append({'name':name,'path':str(output),'resolution_px':resolution,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
                            finally:
                                viewport.camera_path='/Stage1_Overview';viewport.resolution=(1920,1080)
                            answer={'images':images,'simulation_time_s':timeline.get_current_time(),'scope':'viewport previews only; not native sensor bag images','scene':scene.inspect(stage,timeline.get_current_time())}
                            scene.write_json(str(directory.relative_to(S)/'report.json'),answer)
                        elif action=='stage1_record':
                            assert not timeline.is_playing()
                            collection=S/'collection_20260918_v1';validation=S/'validation_20260918_v1'
                            for directory in (validation,collection):
                                assert directory.resolve(strict=True)==directory
                                if str(directory) not in sys.path:sys.path.insert(0,str(directory))
                            import record_runtime
                            status(status='stage1_record',timeline_playing=False,recording=True,case=req.get('case'),session_id=session_id)
                            answer=await record_runtime.run(stage,timeline,viewport,req)
                        elif action=='collection_probe':
                            assert not timeline.is_playing()
                            collection=S/'collection_20260918_v1'
                            assert collection.resolve(strict=True)==collection
                            if str(collection) not in sys.path:sys.path.insert(0,str(collection))
                            import record_probe
                            status(status='collection_probe',timeline_playing=False,recording=False,case=req.get('case'),session_id=session_id)
                            answer=await record_probe.run(stage,timeline,viewport,req)
                        elif action=='camera_time_trial':
                            assert not timeline.is_playing()
                            import importlib.util
                            validation=S/'validation_20260918_v1'
                            assert validation.resolve(strict=True)==validation
                            if str(validation) not in sys.path:sys.path.insert(0,str(validation))
                            module_spec=importlib.util.spec_from_file_location('stage1_camera_time_validation',validation/'runtime.py')
                            validation_module=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(validation_module)
                            status(status='camera_time_trial',timeline_playing=False,recording=True,case=req.get('case'),session_id=session_id)
                            answer=await validation_module.run(stage,timeline,viewport,req)
                        elif action=='pause':timeline.pause();answer={'paused':True}
                        else:raise ValueError('Unsupported stage1 action')
                        atomic_replace_json(M,RUN/'response.json',{'request_id':rid,'action':action,'status':'complete','result':answer})
                    except Exception as exc:
                        timeline.pause();atomic_replace_json(M,RUN/'response.json',{'request_id':rid,'status':'failed','error':str(exc),'traceback':traceback.format_exc()})
            if time.monotonic()-previous>3:
                status(status='stage1_open',scene=str(scene.path('stages/scene.usda')),timeline_playing=timeline.is_playing(),recording=False,
                    simulation_time_s=timeline.get_current_time(),session_id=session_id,viewport_camera=str(viewport.camera_path),
                    robots=['drone_1'],cameras=['cam_01','cam_02','cam_03'],motion='USD waypoint interpolation',recorder_implemented=True,recorder_validated=False)
                previous=time.monotonic()
    except Exception as exc:
        status(status='bootstrap_failed',error=str(exc),traceback=traceback.format_exc(),timeline_playing=False,recording=False)
        print(traceback.format_exc(),flush=True);omni.kit.app.get_app().post_quit(1)

_stage1_task=asyncio.ensure_future(main())
