"""Metadata-only timing checks; these cannot certify the time of image pixels."""
import math


def frame_clock_checks(stamp_ns, metadata, state, tolerance_ns=1000):
    times = (metadata['reference_sim_s'],
             metadata['native_simulation_time']['simulationTime'],
             metadata['clock']['global_s'], state['clock']['global_s'])
    if not all(math.isfinite(float(value)) for value in times):
        raise ValueError('Nonfinite camera clock')
    stamp_ns = int(stamp_ns)
    reference_error = abs(round(float(times[0])*1e9)-stamp_ns)
    native_error = abs(round(float(times[1])*1e9)-stamp_ns)
    mapping_matches = (
        metadata['index'] == state['index']
        and metadata['clock']['step'] == state['clock']['step']
        and int(metadata['stamp_ns']) == int(state['stamp_ns']) == stamp_ns
        and round(float(times[2])*1e9) == round(float(times[3])*1e9) == stamp_ns
    )
    return {
        'mapping_matches_state': mapping_matches,
        'reference_global_error_ns': reference_error,
        'native_annotator_global_error_ns': native_error,
        'metadata_time_pass': mapping_matches and max(reference_error, native_error) <= tolerance_ns,
    }


def assess_run(cameras, clock_checks, counts_match, cleanup_errors):
    metadata_pass = (
        set(cameras) == {'cam_01', 'cam_02', 'cam_03'}
        and clock_checks['strictly_increasing']
        and clock_checks['header_global_max_error_ns'] == 0
        and all(c['images'] > 0 and c['sample_sequence_pass']
                and c['metadata_time_failures'] == 0 for c in cameras.values())
    )
    return {
        'camera_metadata_time_pass': metadata_pass,
        'camera_content_time_status': 'not_checked_qr_removed',
        'camera_content_time_verified': None,
        # The old field included an independent QR check. Do not relabel it as a pass.
        'camera_time_pass': None,
        'transport_pass': bool(counts_match) and not cleanup_errors,
        'lidar_acquisition_time_verified': False,
        'synchronized_dataset_pass': False,
    }
