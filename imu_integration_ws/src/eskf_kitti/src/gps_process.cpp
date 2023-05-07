#include "eskf_kitti/eskf/gps_process.h"
#include <glog/logging.h>

GPSProcess::GPSProcess(const Eigen::Vector3d &I_p_Gps) : I_p_Gps_(I_p_Gps) {}

bool GPSProcess::UpdateStateByGpsPosition(const Eigen::Vector3d &init_lla,
                                          const GPSData gps_data,
                                          State &state) {
    // 后验
    Eigen::Matrix<double, 3, 15> Jac;
    Eigen::Vector3d residual;
    ComputeJacobianAndResidual(init_lla, gps_data, state, &Jac, &residual);

    const Eigen::Matrix3d &V = gps_data.cov;

    const Eigen::MatrixXd &P = state.cov;
    const Eigen::MatrixXd K =
        P * Jac.transpose() * (Jac * P * Jac.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;

    AddDeltaToState(delta_x, &state);
    const Eigen::MatrixXd I_KH =
        Eigen::Matrix<double, 15, 15>::Identity() - K * Jac;
    state.cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}

void GPSProcess::ComputeJacobianAndResidual(
    const Eigen::Vector3d &init_lla, const GPSData gps_data, const State &state,
    Eigen::Matrix<double, 3, 15> *jacobian, Eigen::Vector3d *residual) {

    const Eigen::Vector3d &G_p_I = state.p;
    const Eigen::Matrix3d &G_R_I = state.q.toRotationMatrix();

    Eigen::Vector3d G_p_Gps;
    ConvertLLAToENU(init_lla, gps_data.lla, &G_p_Gps);

    // Compute residual.
    *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);

    // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    ///????
    jacobian->block<3, 3>(0, 6) = -G_R_I * skew(I_p_Gps_);
}

void AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x,
                     State *state) {
    state->p += delta_x.block<3, 1>(0, 0);
    state->v += delta_x.block<3, 1>(3, 0);
    state->ba += delta_x.block<3, 1>(9, 0);
    state->bg += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {

        Eigen::Matrix3d delta_R =
            Eigen::Matrix3d::Identity() - skew(delta_x.block<3, 1>(6, 0));
        Eigen::Quaterniond dq = Eigen::Quaterniond(delta_R);
        dq = dq.normalized();
        state->q *= dq;
    }
}