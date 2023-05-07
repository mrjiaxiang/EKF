#include "eskf_kitti/eskf/initializer.h"
#include <glog/logging.h>

Initializer::Initializer(const Eigen::Vector3d &init_I_p_Gps)
    : init_I_p_Gps_(init_I_p_Gps) {}

void Initializer::AddImuData(const IMUData &imu_data) {
    imu_buffer_.push_back(imu_data);
    if (imu_buffer_.size() > kImuDataBufferLength) {
        imu_buffer_.pop_front();
    }
}

bool Initializer::AddGpsPositionData(const GPSData gps_data, State *state) {
    if (imu_buffer_.size() < kImuDataBufferLength) {
        LOG(INFO) << "imu data is not enough.";
        return false;
    }
    const IMUData last_imu = imu_buffer_.back();
    if (std::abs(gps_data.stamp - last_imu.stamp) > 0.5) {
        LOG(ERROR) << "[AddGpsPositionData]: Gps and imu timestamps are not "
                      "synchronized!";
        return false;
    }

    state->time_stamp = last_imu.stamp;
    state->imu_data = last_imu;

    state->p.setZero();
    state->v.setZero();

    Eigen::Matrix3d G_R_I;

    if (!ComputeG_R_IFromImuData(&G_R_I)) {
        LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_R_I!";
        return false;
    }

    state->q = Eigen::Quaterniond(G_R_I);

    state->ba.setZero();
    state->bg.setZero();

    // Set covariance.
    state->cov.setZero();
    state->cov.block<3, 3>(0, 0) =
        100. * Eigen::Matrix3d::Identity(); // position std: 10 m
    state->cov.block<3, 3>(3, 3) =
        100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
    // roll pitch std 10 degree.
    state->cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. *
                                   kDegreeToRadian *
                                   Eigen::Matrix2d::Identity();
    state->cov(8, 8) =
        100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
    // Acc bias.
    state->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

    return true;
}

bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d *G_R_I) {
    Eigen::Vector3d sum_acc(0.0, 0.0, 0.0);
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data.acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data.acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc =
        (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > kAccStdLimit) {
        LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: "
                     << std_acc.transpose();
        return false;
    }

    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis.
    Eigen::Vector3d x_axis =
        Eigen::Vector3d::UnitX() -
        z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis.
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    *G_R_I = I_R_G.transpose();

    return true;
}