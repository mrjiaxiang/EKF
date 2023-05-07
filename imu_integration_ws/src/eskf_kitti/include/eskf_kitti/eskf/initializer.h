#pragma once

#include <deque>
#include <eskf_kitti/eskf/data_types.h>

constexpr int kImuDataBufferLength = 100;
constexpr int kAccStdLimit = 3.;

class Initializer {
  public:
    Initializer(const Eigen::Vector3d &init_I_p_Gps);
    void AddImuData(const IMUData &imu_data);

    bool AddGpsPositionData(const GPSData gps_data, State *state);

  private:
    bool ComputeG_R_IFromImuData(Eigen::Matrix3d *G_R_I);
    Eigen::Vector3d init_I_p_Gps_;
    std::deque<IMUData> imu_buffer_;
};