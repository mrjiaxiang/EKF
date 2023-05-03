#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

struct IMUData {
    double time = 0.0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};
