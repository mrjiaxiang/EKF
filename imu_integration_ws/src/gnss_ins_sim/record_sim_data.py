# -*- coding: utf-8 -*-
# Filename: publisher_node_no_ros.py

import os
import math
import numpy as np

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

motion_def_path = os.path.abspath('.//demo_motion_def_files')
D2R = math.pi/180

imu_fs = 100.0
gps_fs = 10.0


def gnss_ins_sim_recorder():
    imu_err = {
        'gyro_b': np.array([36.0, 36.0, 36.0]),  # bias deg/hr
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0.0, 0.0, 0.0]),
        # gyro bias instability, deg/hr
        'gyro_b_stability': np.array([0.0, 0.0, 0.0]),
        'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        'gyro_k': np.array([0.98, 0.98, 0.98]),
        'gyro_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
        # accelerometer bias, m/s^2
        'accel_b': np.array([0.1, 0.1, 0.1]),
        # accelerometer velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.05, 0.05, 0.05]),
        'accel_b_stability': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
        # accelerometer bias instability, m/s^2
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        'accel_k': np.array([0.98, 0.98, 0.98]),
        'accel_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),

        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0,
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }

    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    sim = ins_sim.Sim(
        [imu_fs, imu_fs, imu_fs],
        motion_def_path+"//recorder_gnss_ins_sim_speedup.csv",
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    # recorder_gnss_ins_sim_speedup
    sim.run()
    sim.results('./data/recorder_gnss_ins_sim_speedup')
    sim.plot(['ad_accel', 'ad_gyro'])


if __name__ == "__main__":
    gnss_ins_sim_recorder()
