#!/usr/bin/env python3
import copy
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    'prepare_realflight', Path(__file__).resolve().parents[1] / 'scripts/prepare_realflight.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class RealflightConfigTest(unittest.TestCase):
    def setUp(self):
        angle = math.radians(15)
        self.r_ib = np.array([[math.cos(angle), 0, -math.sin(angle)],
                             [0, 1, 0], [math.sin(angle), 0, math.cos(angle)]])
        self.config = {'mapping': {
            'extrinsic_est_en': False,
            'imu_body_R': self.r_ib.reshape(-1).tolist(),
            'imu_body_T': [0.2, -0.1, 0.05],
            'extrinsic_R': np.eye(3).reshape(-1).tolist(),
            'extrinsic_T': [-0.011, -0.02329, 0.04412]},
            'publish': {'tf_imu_frame': 'imu'}}

    def test_imu_cloud_and_lidar_ray_use_the_same_body_geometry(self):
        result = prepare.derive(self.config)
        r_bi = np.array(result['cloud_rotation']).reshape(3, 3)
        t_bi = np.array(result['cloud_translation'])
        # A body point transformed into I must be recovered in B.
        point_b = np.array([3., -2., 1.])
        point_i = self.r_ib @ point_b + np.array(self.config['mapping']['imu_body_T'])
        np.testing.assert_allclose(t_bi + r_bi @ point_i, point_b, atol=1e-12)
        lidar_i = np.array(self.config['mapping']['extrinsic_T'])
        sensor_b = np.array([result['lidar_' + axis] for axis in 'xyz'])
        np.testing.assert_allclose(t_bi + r_bi @ lidar_i, sensor_b, atol=1e-12)
        self.assertAlmostEqual(result['lidar_pitch'], math.radians(15))
        self.assertEqual(result['perception_utils'], {key: result[key] for key in result['perception_utils']})

    def test_rejects_dynamic_extrinsics_invalid_rotation_and_disabled_cloud(self):
        for field, value in [('extrinsic_est_en', True), ('imu_body_R', [0.] * 9),
                             ('extrinsic_T', [float('nan'), 0, 0]),
                             ('imu_body_T', [0., 0.])]:
            config = copy.deepcopy(self.config)
            config['mapping'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                prepare.derive(config)
        config = copy.deepcopy(self.config)
        config['publish']['scan_bodyframe_pub_en'] = False
        with self.assertRaises(ValueError):
            prepare.derive(config)

    def test_roll_yaw_and_pitch_reconstruct_full_mount_rotation(self):
        for roll, pitch, yaw in [(0.3, -0.4, 0.5), (0., math.pi / 2, 0.7), (0., -math.pi / 2, 0.7)]:
            def rotation(r, p, y):
                rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
                ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
                rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
                return rz @ ry @ rx
            expected = rotation(roll, pitch, yaw)
            config = copy.deepcopy(self.config)
            config['mapping']['imu_body_R'] = expected.T.reshape(-1).tolist()
            result = prepare.derive(config)
            actual = rotation(result['lidar_roll'], result['lidar_pitch'], result['lidar_yaw'])
            np.testing.assert_allclose(actual, expected, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
