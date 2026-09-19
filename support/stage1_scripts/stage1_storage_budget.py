"""Count original campaign plus the complete new stage1 writable subtree."""
import math
from pathlib import Path
import shutil
import time

ROOT = Path('/mnt/DATA/workspace/ws_minho/mro_1')


def check_storage_budget(root, incoming_bytes=0):
    from mro_runtime.paths import read_json
    from storage_budget import tracked_bytes, evaluate_budget
    started = time.monotonic()
    root = Path(root)
    if root != ROOT or root.is_symlink() or root.resolve(strict=True) != ROOT:
        raise ValueError('Unexpected stage1 storage source root')
    project = root / 'stage1'
    if project.is_symlink() or project.resolve(strict=True) != project:
        raise ValueError('Unexpected stage1 storage tree')
    policy = read_json(root, 'config/storage_budget.json')
    required = {'outputs', 'runtime', 'scripts', 'config', 'docs', 'manifests'}
    if policy.get('schema_version') != 1 or set(policy.get('tracked_directories', [])) != required:
        raise ValueError('Unexpected original campaign storage policy')
    if policy['max_growth_bytes'] > 2 * 1024**4 or policy['reserved_free_bytes'] < math.ceil(1.13 * 1024**4):
        raise ValueError('Storage policy exceeds the user-authorized bounds')
    # Single traversal deduplicates hard links across old and new trees. A
    # separate count is diagnostic only; no new allowance/baseline is introduced.
    total, count = tracked_bytes(root, list(policy['tracked_directories']) + ['stage1'])
    stage1_bytes, stage1_count = tracked_bytes(root, ['stage1'])
    result = evaluate_budget(policy, total, shutil.disk_usage(root).free, incoming_bytes)
    result.update(tracked_regular_files=count, stage1_tracked_bytes=stage1_bytes,
                  stage1_regular_files=stage1_count, original_baseline_preserved=True,
                  tracked_directories=list(policy['tracked_directories']) + ['stage1'],
                  accounting_wall_seconds=time.monotonic()-started)
    return result
