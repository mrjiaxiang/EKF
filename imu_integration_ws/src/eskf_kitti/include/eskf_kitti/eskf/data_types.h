#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>

constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;

struct IMUData {
    /* data */
    double stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct GPSData {
    /* data */
    double stamp;
    Eigen::Vector3d lla;
    Eigen::Matrix3d cov;
};

struct State {
    double time_stamp;

    Eigen::Quaterniond q;
    Eigen::Vector3d v;
    Eigen::Vector3d p;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    Eigen::Matrix<double, 15, 15> cov;

    Eigen::Vector3d lla;
    IMUData imu_data;
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0;
    return matrix;
}

inline void ConvertENUToLLA(const Eigen::Vector3d &init_lla,
                            const Eigen::Vector3d &point_enu,
                            Eigen::Vector3d *point_lla) {
    GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                            point_lla->data()[0], point_lla->data()[1],
                            point_lla->data()[2]);
}