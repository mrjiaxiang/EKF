#include "eskf_kitti/eskf/imu_gps_process.h"
#include <glog/logging.h>

IMUGPSProcess::IMUGPSProcess(const double acc_noise, const double gyro_noise,
                             const double acc_bias_noise,
                             const double gyro_bias_noise,
                             const Eigen::Vector3d &I_p_Gps)
    : initialized_(false) {
    initializer_ptr_ = std::make_shared<Initializer>(I_p_Gps);
    imu_process_ptr_ = std::make_shared<IMUProcess>(
        acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise,
        Eigen::Vector3d(0., 0., -9.81007));
    gps_process_ptr_ = std::make_shared<GPSProcess>(I_p_Gps);
}

bool IMUGPSProcess::ProcessImuData(const IMUData imu_data, State *fused_state) {
    if (!initialized_) {
        initializer_ptr_->AddImuData(imu_data);
        LOG(INFO) << "add imu data.";
        return false;
    }
    imu_process_ptr_->Predict(state_.imu_data, imu_data, state_);

    ConvertENUToLLA(init_lla_, state_.p, &(state_.lla));
    *fused_state = state_;
    return true;
}

bool IMUGPSProcess::ProcessGpsPositionData(const GPSData gps_data) {
    if (!initialized_) {
        if (!initializer_ptr_->AddGpsPositionData(gps_data, &state_)) {
            return false;
        }

        init_lla_ = gps_data.lla;
        initialized_ = true;
        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    gps_process_ptr_->UpdateStateByGpsPosition(init_lla_, gps_data, state_);
    return true;
}