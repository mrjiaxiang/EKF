#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

#include "eskf_kitti/eskf/data_types.h"
#include "eskf_kitti/eskf/gps_process.h"
#include "eskf_kitti/eskf/imu_process.h"
#include "eskf_kitti/eskf/initializer.h"

class IMUGPSProcess {
  public:
    IMUGPSProcess(const double acc_noise, const double gyro_noise,
                  const double acc_bias_noise, const double gyro_bias_noise,
                  const Eigen::Vector3d &I_p_Gps);

    bool ProcessImuData(const IMUData imu_data_ptr, State *fused_state);

    bool ProcessGpsPositionData(const GPSData gps_data_ptr);
    inline void ConvertENUToLLA(const Eigen::Vector3d &init_lla,
                                const Eigen::Vector3d &point_enu,
                                Eigen::Vector3d *point_lla) {

        local_cartesian_.Reset(init_lla(0), init_lla(1), init_lla(2));
        local_cartesian_.Reverse(point_enu(0), point_enu(1), point_enu(2),
                                 point_lla->data()[0], point_lla->data()[1],
                                 point_lla->data()[2]);
    }

  private:
    std::shared_ptr<Initializer> initializer_ptr_;
    std::shared_ptr<IMUProcess> imu_process_ptr_;
    std::shared_ptr<GPSProcess> gps_process_ptr_;

    bool initialized_;
    Eigen::Vector3d init_lla_; // The initial reference gps point.
    State state_;

    GeographicLib::LocalCartesian local_cartesian_;
};
