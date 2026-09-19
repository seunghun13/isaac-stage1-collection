"""Offline source regression and synthetic tests. Never imports a remote launcher."""
from pathlib import Path
import ast
import copy
import datetime
import hashlib
import io
import json
import sys
import unittest

D = Path(__file__).resolve().parent
W = D.parent
origin = json.loads((D/'ORIGIN.json').read_text(encoding='utf-8'))
checks = {}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

checks['original_local_sources_preserved'] = all(
    digest(W/item['source']) == item['sha256'] for item in origin['files'].values())
unchanged = ('record_render.py', 'record_worker.py', 'record_driver.py',
             'record_lidar.py', 'record_probe.py', 'waypoints.json',
             'stage1_scene.py', 'view_hangar_bootstrap.py')
checks['render_ros_lidar_driver_scene_route_byte_identical'] = all(
    digest(D/name) == origin['files'][name]['sha256'] for name in unchanged)
syntax = [p.name for p in D.glob('*.py')]
for name in syntax:
    ast.parse((D/name).read_text(encoding='utf-8'), filename=name)
checks['all_python_sources_parse'] = True

old = ast.parse((W/origin['files']['record_runtime.py']['source']).read_text(encoding='utf-8'))
new = ast.parse((D/'record_runtime.py').read_text(encoding='utf-8'))

def named(tree, name):
    return next(n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and n.name == name)

def dump(tree):
    return ast.dump(tree, include_attributes=False)

for name in ('Emitter', 'pointcloud', 'clock', 'render', 'on_frame'):
    checks[name+'_unchanged_ast'] = dump(named(old, name)) == dump(named(new, name))

def acquisition_loop(tree):
    return next(n for n in ast.walk(named(tree, 'run'))
                if isinstance(n, ast.For) and ast.unparse(n.iter) == "range(spec['ticks'] + 1)")

class RemoveQrOnly(ast.NodeTransformer):
    def visit_Expr(self, node):
        if isinstance(node.value, ast.Call) and ast.unparse(node.value.func) in ('markers.update', 'board.update'):
            return None
        return self.generic_visit(node)

    def visit_Assign(self, node):
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('decoded', 'match'):
            return None
        return self.generic_visit(node)

    def visit_AugAssign(self, node):
        if ast.unparse(node.target).startswith("report['online_qr']"):
            return None
        return self.generic_visit(node)

    def visit_Call(self, node):
        node = self.generic_visit(node)
        if isinstance(node.func, ast.Name) and node.func.id == 'dict':
            node.keywords = [k for k in node.keywords if k.arg != 'online_qr']
        return node

checks['entire_acquisition_loop_unchanged_except_qr'] = (
    dump(RemoveQrOnly().visit(copy.deepcopy(acquisition_loop(old)))) == dump(acquisition_loop(new)))
for variable in ('warm', 'cam'):
    # Warmup loop and camera RenderProduct/annotator construction must retain their order and settings.
    def selected(tree):
        return next(n for n in ast.walk(named(tree, 'run')) if isinstance(n, ast.For)
                    and isinstance(n.target, ast.Name) and n.target.id == variable
                    and (variable == 'warm' or ast.unparse(n.iter) == 'cams'))
    checks[variable+'_setup_loop_unchanged_ast'] = dump(selected(old)) == dump(selected(new))
for tree_name, tree in (('runtime', new), ('analyzer', ast.parse((D/'analyze_bag.py').read_text(encoding='utf-8')))):
    checks[tree_name+'_no_qr_imports_or_constructors'] = not any(
        isinstance(n, ast.ImportFrom) and n.module in ('scene_markers', 'drone_board', 'marker_codec', 'board_vision')
        or isinstance(n, ast.Call) and ast.unparse(n.func) in ('SceneMarkers', 'DroneBoard', 'decode_marker', 'detect_board')
        for n in ast.walk(tree))

# Execute the actual residual-fixture guard against a tiny stage API double.
guard_module = ast.Module(body=[copy.deepcopy(named(new, 'assert_qr_free_stage'))], type_ignores=[])
namespace = {}
exec(compile(ast.fix_missing_locations(guard_module), '<guard-only>', 'exec'), namespace)
class Prim:
    def __init__(self, path): self.path = path
    def GetPath(self): return self.path
    def GetName(self): return self.path.rsplit('/', 1)[-1]
class Stage:
    def __init__(self, paths): self.paths = paths
    def Traverse(self): return (Prim(p) for p in self.paths)
guard = namespace['assert_qr_free_stage']
guard(Stage(['/World/drone_1/base_link/TrackingCenter', '/MRO_Cameras/cam_01']))
rejected = []
for path in ('/Stage1ValidationMarkers', '/World/drone_1/base_link/TrackingCenter/ValidationTimeBoard_102'):
    try: guard(Stage([path]))
    except ValueError: rejected.append(path)
checks['residual_camera_and_drone_fixtures_rejected'] = len(rejected) == 2

sys.path.insert(0, str(D))
from test_timing_assessment import TimingAssessmentTests
stream = io.StringIO()
tests = unittest.TextTestRunner(stream=stream, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(TimingAssessmentTests))
checks['synthetic_timing_regressions_pass'] = tests.wasSuccessful()
report = {'status': 'local_checks_passed' if all(checks.values()) else 'failed',
          'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'checks': checks, 'synthetic_tests': tests.testsRun, 'test_output': stream.getvalue(),
          'python_files_parsed': len(syntax), 'source_sha256': {n: digest(D/n) for n in sorted(syntax)},
          'server_contacted': False, 'deployed': False, 'isaac_runtime_tested': False,
          'image_content_time_equivalence_verified': False,
          'lidar_acquisition_time_verified': False}
(D/'LOCAL_VERIFICATION.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256', 'test_output')}, indent=2))
if not all(checks.values()):
    print(stream.getvalue())
    raise SystemExit(1)
