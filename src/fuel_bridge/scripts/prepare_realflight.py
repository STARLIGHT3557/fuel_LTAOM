#!/usr/bin/env python3
"""Derive cloud and ray extrinsics from the onboard Faster-LIO configuration."""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml


def vector(mapping, key, size):
    value = np.asarray(mapping[key], dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{key} requires {size} finite values')
    return value


def rotation(mapping, key):
    value = vector(mapping, key, 9).reshape(3, 3)
    if (not np.allclose(value.T @ value, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(np.linalg.det(value), 1.0, abs_tol=1e-6)):
        raise ValueError(f'{key} must be a proper rotation matrix')
    return value


def derive(config):
    mapping = config['mapping']
    publish = config.get('publish', {})
    if mapping.get('extrinsic_est_en', True) is not False:
        raise ValueError('mapping/extrinsic_est_en must be false: fixed cloud/ray extrinsics required')
    if publish.get('scan_bodyframe_pub_en', True) is not True or publish.get('scan_publish_en', True) is not True:
        raise ValueError('Enable publish/scan_publish_en and publish/scan_bodyframe_pub_en')

    # Faster-LIO: p_I = R_IB p_B + t_IB; p_I = R_IL p_L + t_IL.
    # cloud_registered_body is actually in I (the selected LIO IMU frame).
    r_ib = rotation(mapping, 'imu_body_R')
    t_ib = vector(mapping, 'imu_body_T', 3)
    r_il = rotation(mapping, 'extrinsic_R')
    t_il = vector(mapping, 'extrinsic_T', 3)
    r_bi = r_ib.T
    t_bi = -r_bi @ t_ib
    r_bl = r_bi @ r_il
    t_bl = r_bi @ (t_il - t_ib)

    # R_BL = Rz(yaw) Ry(pitch) Rx(roll), shared by bridge and FOV checks.
    pitch = math.atan2(-r_bl[2, 0], math.hypot(r_bl[0, 0], r_bl[1, 0]))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(r_bl[2, 1], r_bl[2, 2])
        yaw = math.atan2(r_bl[1, 0], r_bl[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-r_bl[0, 1], r_bl[1, 1])
    frame = publish.get('tf_imu_frame', 'imu')
    if not isinstance(frame, str) or not frame or frame.startswith('/'):
        raise ValueError('publish/tf_imu_frame must be a nonempty frame name without a leading slash')
    sensor = dict(zip(('lidar_x', 'lidar_y', 'lidar_z'), map(float, t_bl)))
    sensor.update(lidar_roll=roll, lidar_pitch=pitch, lidar_yaw=yaw)
    return dict(sensor, cloud_frame=frame, cloud_translation=t_bi.tolist(),
                cloud_rotation=r_bi.reshape(-1).tolist(), perception_utils=dict(sensor))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('lio_config', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    try:
        with args.lio_config.open() as stream:
            result = derive(yaml.safe_load(stream))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        parser.exit(1, f'FAST-LIO sensor configuration rejected: {error}\n')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('# Derived from ' + str(args.lio_config.resolve()) + '\n'
                           + yaml.safe_dump(result, sort_keys=False))
    print(f'FAST-LIO IMU cloud frame: {result["cloud_frame"]}; sensor config: {args.output}')


if __name__ == '__main__':
    main()
