"""Finite control for the isolated stage1 viewer; no play or recording command."""

import argparse
import json
import math
from pathlib import Path
import re
import sys
import time
import uuid


M = Path('/mnt/DATA/workspace/ws_minho/mro_1')
LAUNCH_PATTERN = re.compile(r'^[0-9]{8}T[0-9]{6}Z_[0-9a-f]{10}$')
ACTIONS = ('status', 'inspect', 'pause', 'reset', 'seek', 'capture-views')


def bounded_path(stage1, path):
    root = stage1.resolve()
    actual = path.resolve()
    if actual == root or not actual.is_relative_to(root):
        raise RuntimeError('Control path escapes the stage1 directory')
    return actual


def control(action, seconds, read_json, atomic_replace_json, *, root=M,
            monotonic=time.monotonic, sleep=time.sleep):
    """Read-only status or a single request through the existing viewer protocol."""
    if action not in ACTIONS:
        raise ValueError('Unsupported action')
    stage1 = root / 'stage1'
    if stage1.resolve() == root.resolve() or not stage1.resolve().is_relative_to(root.resolve()):
        raise RuntimeError('stage1 directory is outside the project')

    def read(path):
        return read_json(root, bounded_path(stage1, path))

    cfg_path = stage1 / 'config/viewing_session.json'
    cfg = read(cfg_path)
    launch = cfg.get('consumed_by_launch_id')
    if not isinstance(launch, str) or not LAUNCH_PATTERN.fullmatch(launch):
        raise RuntimeError('No valid stage1 launch ID in viewing_session.json')
    run = bounded_path(stage1, stage1 / 'runtime/run' / launch)
    owner = read(run / 'owner.json')
    stage = read(run / 'stage_status.json')
    if owner.get('launch_id') != launch or stage.get('launch_id') != launch:
        raise RuntimeError('Stage1 owner/stage launch IDs do not match the current configuration')
    if action == 'status':
        result = {'status': 'read_only', 'launch_id': launch, 'run': str(run),
                  'owner': owner, 'stage': stage}
        request_path = bounded_path(stage1, run / 'request.json')
        if request_path.exists():
            prior = read(request_path)
            response_path = bounded_path(stage1, run / 'response.json')
            response = read(response_path) if response_path.exists() else None
            result['last_request'] = prior
            result['last_response'] = response
            result['request_pending'] = response is None or response.get('request_id') != prior.get('request_id')
        else:
            result['request_pending'] = False
        return result
    if owner.get('status') != 'running' or stage.get('status') != 'stage1_open':
        raise RuntimeError('Current owned stage1 viewer is not running with stage1_open status')
    extra = {}
    if action == 'seek':
        if isinstance(seconds, bool) or not isinstance(seconds, (float, int)) or not math.isfinite(seconds):
            raise ValueError('--seconds must be finite')
        report = read(stage1 / 'manifests/scene_build.json')
        arrivals = report.get('trajectory', {}).get('arrival_times_s')
        if not isinstance(arrivals, list) or len(arrivals) < 2:
            raise RuntimeError('Scene report has no valid trajectory arrival schedule')
        if any(isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t) for t in arrivals):
            raise RuntimeError('Scene trajectory contains invalid arrival times')
        if arrivals[0] != 0 or any(a >= b for a, b in zip(arrivals, arrivals[1:])):
            raise RuntimeError('Scene trajectory arrival times are not strictly increasing from zero')
        if not 0 <= seconds <= arrivals[-1]:
            raise ValueError(f'--seconds must be between 0 and {arrivals[-1]}')
        extra['simulation_time_s'] = float(seconds)
    elif seconds is not None:
        raise ValueError('--seconds is only valid with seek')

    request_path = bounded_path(stage1, run / 'request.json')
    response_path = bounded_path(stage1, run / 'response.json')
    # Match the existing request/response protocol: never replace pending work.
    if request_path.exists():
        prior = read(request_path)
        if not prior.get('request_id'):
            raise RuntimeError('Previous stage1 request has no request ID')
        if not response_path.exists() or read(response_path).get('request_id') != prior['request_id']:
            raise RuntimeError('Previous stage1 request is still pending; use status and wait')
    # Check launch again immediately before submitting, protecting a restart race.
    if read(cfg_path).get('consumed_by_launch_id') != launch:
        raise RuntimeError('Stage1 launch changed before submission')
    current_owner, current_stage = read(run / 'owner.json'), read(run / 'stage_status.json')
    if (current_owner.get('launch_id') != launch or current_owner.get('status') != 'running'
            or current_stage.get('launch_id') != launch or current_stage.get('status') != 'stage1_open'):
        raise RuntimeError('Stage1 owner/status changed before submission')
    request = {'action': action.replace('-', '_'), 'request_id': 'stage1_' + uuid.uuid4().hex,
               'expected_launch_id': launch, **extra}
    atomic_replace_json(root, request_path, request)
    result = {'status': 'submitted', 'request': request, 'run': str(run)}
    if action == 'capture-views':
        result['note'] = 'Viewport previews requested; use status to read completion. This does not record a rosbag.'
        return result
    deadline = monotonic() + 15.0
    while monotonic() < deadline:
        if response_path.exists():
            response = read(response_path)
            if response.get('request_id') == request['request_id']:
                result.update(status=response.get('status', 'response_received'), response=response)
                return result
        sleep(min(0.2, max(0.0, deadline - monotonic())))
    result['status'] = 'pending'
    result['note'] = 'No matching response within 15 seconds. The request is preserved; use status before submitting another.'
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='Inspect or control the isolated stage1 viewer. No play command is provided.')
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ACTIONS:
        command_parser = sub.add_parser(command)
        if command == 'seek':
            command_parser.add_argument('--seconds', required=True, type=float)
    args = parser.parse_args(argv)
    sys.path[:0] = [str(M), str(M / 'scripts')]
    from mro_runtime.paths import read_json, atomic_replace_json
    try:
        result = control(args.command, getattr(args, 'seconds', None), read_json, atomic_replace_json)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1 if result.get('status') == 'failed' else 0
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(json.dumps({'status': 'refused', 'error': str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
