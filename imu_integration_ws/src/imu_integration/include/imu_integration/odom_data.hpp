#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

struct OdomData {
    double time = 0.0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
};