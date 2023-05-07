#pragma once

#include "eskf_kitti/eskf/data_types.h"

class IMUProcess {
  public:
    IMUProcess(const double acc_noise, const double gyro_noise,
               const double acc_bias_noise, const double gyro_bias_noise,
               const Eigen::Vector3d &gravity);

    void Predict(const IMUData prev_imu, const IMUData curr_imu, State &state);

  private:
    const double acc_noise_;
    const double gyro_noise_;
    const double acc_bias_noise_;
    const double gyro_bias_noise_;

    const Eigen::Vector3d gravity_;
};