#include "eskf_kitti/eskf/imu_process.h"
#include "eskf_kitti/eskf/data_types.h"

#include <glog/logging.h>

IMUProcess::IMUProcess(const double acc_noise, const double gyro_noise,
                       const double acc_bias_noise,
                       const double gyro_bias_noise,
                       const Eigen::Vector3d &gravity)
    : acc_noise_(acc_noise), gyro_noise_(gyro_noise),
      acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
      gravity_(gravity) {}

void IMUProcess::Predict(const IMUData prev_imu, const IMUData curr_imu,
                         State &state) {
    const double delta_t = curr_imu.stamp - prev_imu.stamp;
    const double delta_t2 = delta_t * delta_t;

    State last_state = state;

    // 去除bias 中值积分
    const Eigen::Vector3d acc_unbias =
        0.5 * (prev_imu.acc + curr_imu.acc) - last_state.ba;
    const Eigen::Vector3d gyro_unbias =
        0.5 * (prev_imu.gyro + curr_imu.gyro) - last_state.bg;
    // TODO：先验
    state.p = last_state.p + last_state.v * delta_t +
              0.5 * (last_state.q.toRotationMatrix() * acc_unbias + gravity_) *
                  delta_t2;
    state.v =
        last_state.v +
        (last_state.q.toRotationMatrix() * acc_unbias + gravity_) * delta_t;

    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;

    double angular_delta_mag = delta_angle_axis.norm();
    Eigen::Vector3d angular_delta_dir = delta_angle_axis.normalized();

    double angular_delta_cos = cos(angular_delta_mag / 2.0);
    double angular_delta_sin = sin(angular_delta_mag / 2.0);

    Eigen::Quaterniond dq(angular_delta_cos,
                          angular_delta_sin * angular_delta_dir.x(),
                          angular_delta_sin * angular_delta_dir.y(),
                          angular_delta_sin * angular_delta_dir.z());

    state.q = last_state.q * dq;

    Eigen::Matrix<double, 15, 15> Fx =
        Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6) =
        -state.q.toRotationMatrix() * skew(acc_unbias) * delta_t;
    Fx.block<3, 3>(3, 9) = -state.q.toRotationMatrix() * delta_t;

    Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(),
                                             delta_angle_axis.normalized())
                               .toRotationMatrix()
                               .transpose();
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) =
        delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) =
        delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

    state.cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

    state.time_stamp = curr_imu.stamp;
    state.imu_data = curr_imu;
}