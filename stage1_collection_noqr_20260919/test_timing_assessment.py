"""Synthetic regressions for truthful timing reports; no Isaac, ROS or SSH."""
import copy
import math
import unittest
from timing_assessment import frame_clock_checks, assess_run


def sample(index):
    # The observed pilot clock advances by float32(1/60), not exact decimal 1/60.
    global_s = (index+2)*0.01666666753590107
    stamp = round(global_s*1e9)
    state = {'index': index, 'stamp_ns': stamp,
             'clock': {'global_s': global_s, 'step': index+2}}
    metadata = dict(copy.deepcopy(state), reference_sim_s=global_s,
                    native_simulation_time={'simulationTime': global_s})
    return stamp, metadata, state


def summary():
    cameras = {cid: {'images': 451, 'sample_sequence_pass': True, 'metadata_time_failures': 0}
               for cid in ('cam_01', 'cam_02', 'cam_03')}
    clocks = {'strictly_increasing': True, 'header_global_max_error_ns': 0}
    return cameras, clocks


class TimingAssessmentTests(unittest.TestCase):
    def test_thirty_second_synthetic_clock_keeps_all_451_sample_headers(self):
        for index in range(0, 1801, 4):
            with self.subTest(index=index):
                self.assertTrue(frame_clock_checks(*sample(index))['metadata_time_pass'])

    def test_one_step_old_native_or_reference_time_fails(self):
        for field in ('native', 'reference'):
            stamp, metadata, state = sample(60)
            if field == 'native':
                metadata['native_simulation_time']['simulationTime'] -= 1/60
            else:
                metadata['reference_sim_s'] -= 1/60
            with self.subTest(field=field):
                self.assertFalse(frame_clock_checks(stamp, metadata, state)['metadata_time_pass'])

    def test_wrong_clock_join_or_header_fails(self):
        for field in ('index', 'step', 'stamp', 'global'):
            stamp, metadata, state = sample(60)
            if field == 'index': metadata['index'] -= 4
            elif field == 'step': metadata['clock']['step'] -= 1
            elif field == 'stamp': stamp += 1
            else: metadata['clock']['global_s'] -= 1/60
            with self.subTest(field=field):
                self.assertFalse(frame_clock_checks(stamp, metadata, state)['metadata_time_pass'])

    def test_nonfinite_clocks_cannot_pass(self):
        for value in (math.nan, math.inf, -math.inf):
            stamp, metadata, state = sample(60)
            metadata['reference_sim_s'] = value
            with self.assertRaises(ValueError): frame_clock_checks(stamp, metadata, state)

    def test_good_metadata_never_certifies_image_pixels_or_lidar(self):
        # Identical metadata can accompany stale pixels; no QR-free metadata check detects that.
        result = assess_run(*summary(), True, [])
        self.assertTrue(result['camera_metadata_time_pass'])
        self.assertTrue(result['transport_pass'])
        self.assertIsNone(result['camera_content_time_verified'])
        self.assertIsNone(result['camera_time_pass'])
        self.assertFalse(result['lidar_acquisition_time_verified'])
        self.assertFalse(result['synchronized_dataset_pass'])

    def test_missing_duplicate_or_bad_timing_sample_fails_metadata_summary(self):
        for fault in ('missing_camera', 'sequence', 'frame', 'reversal', 'header'):
            cameras, clocks = summary()
            if fault == 'missing_camera': cameras.pop('cam_03')
            elif fault == 'sequence': cameras['cam_01']['sample_sequence_pass'] = False
            elif fault == 'frame': cameras['cam_01']['metadata_time_failures'] = 1
            elif fault == 'reversal': clocks['strictly_increasing'] = False
            else: clocks['header_global_max_error_ns'] = 1
            with self.subTest(fault=fault):
                self.assertFalse(assess_run(cameras, clocks, True, [])['camera_metadata_time_pass'])

    def test_loss_and_cleanup_failure_remain_transport_failures(self):
        for counts, errors in ((False, []), (True, ['detach failed'])):
            self.assertFalse(assess_run(*summary(), counts, errors)['transport_pass'])


if __name__ == '__main__':
    unittest.main()
