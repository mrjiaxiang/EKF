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

def gnss_ins_sim_publisher():

    imu_err = {'gyro_b': np.array([0.0, 0.0, 0.0]),#bias deg/hr
               'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,# gyro angle random walk, deg/rt-hr
               'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,#gyro bias instability, deg/hr
               'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
               'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),# accelerometer bias, m/s^2
               'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,# accelerometer velocity random walk, m/s/rt-hr
               'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
               'accel_b_corr': np.array([200.0, 200.0, 200.0]),# accelerometer bias instability, m/s^2
               'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0
              }
    # imu_err = 'mid_accuracy'
    imu = imu_model.IMU(accuracy=imu_err,axis=9,gps=True)

    sim = ins_sim.Sim(
        [imu_fs,gps_fs,imu_fs],
        motion_def_path+"//demo.csv",
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    sim.run()
    sim.results('./data/demo')
    sim.plot(['ad_accel','ad_gyro'])

if __name__ =='__main__':
    gnss_ins_sim_publisher()