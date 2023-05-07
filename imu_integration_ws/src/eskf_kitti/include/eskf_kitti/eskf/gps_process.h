#pragma once

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <eskf_kitti/eskf/data_types.h>

class GPSProcess {
  public:
    GPSProcess(const Eigen::Vector3d &I_p_Gps);

    bool UpdateStateByGpsPosition(const Eigen::Vector3d &init_lla,
                                  const GPSData gps_data, State &state);

    inline void ConvertLLAToENU(const Eigen::Vector3d &init_lla,
                                const Eigen::Vector3d &point_lla,
                                Eigen::Vector3d *point_enu) {

        local_cartesian_.Reset(init_lla(0), init_lla(1), init_lla(2));
        local_cartesian_.Forward(point_lla(0), point_lla(1), point_lla(2),
                                 point_enu->data()[0], point_enu->data()[1],
                                 point_enu->data()[2]);
    }

  private:
    void ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                    const GPSData gps_data, const State &state,
                                    Eigen::Matrix<double, 3, 15> *jacobian,
                                    Eigen::Vector3d *residual);

    const Eigen::Vector3d I_p_Gps_;
    GeographicLib::LocalCartesian local_cartesian_;
};

void AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x, State *state);