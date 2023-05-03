#include "imu_integration/estimator.hpp"
#include <glog/logging.h>

Estimator::Estimator()
    : private_nh_("~"), initialized_(false), G_(0, 0, -9.81),
      angular_vel_bias_(0.0, 0.0, 0.0), linear_acc_bias_(0.0, 0.0, 0.0) {}

Estimator::~Estimator() {
    gt_ofs_.close();
    imu_ofs_.close();
}

void Estimator::Init() {
    private_nh_.param("imu/topic_name", imu_config_.topic_name,
                      std::string("/sim/sensor/imu"));

    imu_sub_ = private_nh_.subscribe(imu_config_.topic_name, 1000,
                                     &Estimator::imu_callback, this);

    private_nh_.param("imu/gravity/x", imu_config_.gravity.x, 0.0);
    private_nh_.param("imu/gravity/y", imu_config_.gravity.y, 0.0);
    private_nh_.param("imu/gravity/z", imu_config_.gravity.z, -9.81);

    G_.x() = imu_config_.gravity.x;
    G_.y() = imu_config_.gravity.y;
    G_.z() = imu_config_.gravity.z;

    private_nh_.param("imu/bias/angular_velocity/x",
                      imu_config_.bias.angular_velocity.x, 0.0);
    private_nh_.param("imu/bias/angular_velocity/y",
                      imu_config_.bias.angular_velocity.y, 0.0);
    private_nh_.param("imu/bias/angular_velocity/z",
                      imu_config_.bias.angular_velocity.z, 0.0);
    angular_vel_bias_.x() = imu_config_.bias.angular_velocity.x;
    angular_vel_bias_.y() = imu_config_.bias.angular_velocity.y;
    angular_vel_bias_.z() = imu_config_.bias.angular_velocity.z;

    private_nh_.param("imu/bias/linear_acceleration/x",
                      imu_config_.bias.linear_acceleration.x, 0.0);
    private_nh_.param("imu/bias/linear_acceleration/y",
                      imu_config_.bias.linear_acceleration.y, 0.0);
    private_nh_.param("imu/bias/linear_acceleration/z",
                      imu_config_.bias.linear_acceleration.z, 0.0);
    linear_acc_bias_.x() = imu_config_.bias.linear_acceleration.x;
    linear_acc_bias_.y() = imu_config_.bias.linear_acceleration.y;
    linear_acc_bias_.z() = imu_config_.bias.linear_acceleration.z;

    private_nh_.param("pose/frame_id", odom_config_.frame_id,
                      std::string("inertial"));
    private_nh_.param("pose/topic_name/ground_truth",
                      odom_config_.topic_name.ground_truth,
                      std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/estimation",
                      odom_config_.topic_name.estimation,
                      std::string("/pose/estimation"));

    odom_sub_ = private_nh_.subscribe(odom_config_.topic_name.ground_truth,
                                      1000, &Estimator::odom_callback, this);

    odom_estimation_pub_ = private_nh_.advertise<nav_msgs::Odometry>(
        odom_config_.topic_name.estimation, 500);
    path_pub_ = private_nh_.advertise<nav_msgs::Path>("trajectory_odom", 100);

    gt_ofs_.open("/home/melody/example/EKF/imu_integration_ws/src/csv_to_bag/"
                 "data/gt.txt",
                 std::ios::out | std::ios::trunc);

    imu_ofs_.open("/home/melody/example/EKF/imu_integration_ws/src/csv_to_bag/"
                  "data/imu.txt",
                  std::ios::out | std::ios::trunc);
}

void Estimator::imu_callback(const sensor_msgs::ImuConstPtr &msg) {
    IMUData imu_data;
    imu_data.time = msg->header.stamp.toSec();
    imu_data.linear_acceleration =
        Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);

    imu_data.linear_acceleration =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                        msg->angular_velocity.z);

    imu_data_buff_.push_back(imu_data);
}

void Estimator::odom_callback(const nav_msgs::OdometryConstPtr &odom_msg_ptr) {
    OdomData odom_data;

    odom_data.time = odom_msg_ptr->header.stamp.toSec();

    Eigen::Quaterniond q(odom_msg_ptr->pose.pose.orientation.w,
                         odom_msg_ptr->pose.pose.orientation.x,
                         odom_msg_ptr->pose.pose.orientation.y,
                         odom_msg_ptr->pose.pose.orientation.z);
    Eigen::Vector3d t(odom_msg_ptr->pose.pose.position.x,
                      odom_msg_ptr->pose.pose.position.y,
                      odom_msg_ptr->pose.pose.position.z);

    odom_data.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom_data.pose.block<3, 1>(0, 3) = t;

    SavePose(gt_ofs_, odom_data.pose);

    odom_data.vel = Eigen::Vector3d(odom_msg_ptr->twist.twist.linear.x,
                                    odom_msg_ptr->twist.twist.linear.y,
                                    odom_msg_ptr->twist.twist.linear.z);

    // add new message to buffer:
    odom_data_buff_.push_back(odom_data);
}

bool Estimator::hasData() {

    if (imu_data_buff_.size() < static_cast<size_t>(3)) {
        return false;
    }

    if (!initialized_ && static_cast<size_t>(0) == odom_data_buff_.size()) {
        return false;
    }
    return true;
}

bool Estimator::UpdatePose() {
    if (!initialized_) {
        // 使用odom进行初始化
        OdomData odom_data = odom_data_buff_.back();
        IMUData imu_data = imu_data_buff_.back();

        pose_ = odom_data.pose;
        vel_ = odom_data.vel;

        initialized_ = true;

        odom_data_buff_.clear();
        imu_data_buff_.clear();
        imu_data_buff_.push_back(imu_data);
    } else {
        size_t index_prev = 0;
        size_t index_curr = 1;
        Eigen::Vector3d angular_delta;
        // step 1:获取角增量
        GetAngularDelta(index_curr, index_prev, angular_delta);
        // step 2:更新姿态
        Eigen::Matrix3d R_curr, R_prev;
        UpdateOrientation(angular_delta, R_curr, R_prev);

        // step 3: 速度更新
        double delta_t;
        Eigen::Vector3d velocity_delta;
        GetVelocityDelta(index_curr, index_prev, R_curr, R_prev, delta_t,
                         velocity_delta);

        // step 4:位置更新
        UpdatePosition(delta_t, velocity_delta);

        imu_data_buff_.pop_front();
    }
    return true;
}

bool Estimator::GetAngularDelta(size_t index_curr, size_t index_prev,
                                Eigen::Vector3d &angular_delta) {
    if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
        return false;
    }
    const IMUData imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData imu_data_prev = imu_data_buff_.at(index_prev);

    double delta_t = imu_data_curr.time - imu_data_prev.time;
    Eigen::Vector3d angular_vel_curr =
        GetUnbiasedAngularVel(imu_data_curr.angular_velocity);
    Eigen::Vector3d angular_vel_prev =
        GetUnbiasedAngularVel(imu_data_prev.angular_velocity);

    angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);
    return true;
}

Eigen::Vector3d
Estimator::GetUnbiasedAngularVel(const Eigen::Vector3d &angular_velocity) {
    // 因为陀螺使用的是角增量
    return angular_velocity - angular_vel_bias_;
}

void Estimator::UpdateOrientation(const Eigen::Vector3d &angular_delta,
                                  Eigen::Matrix3d &R_curr,
                                  Eigen::Matrix3d &R_prev) {
    double angular_delta_mag = angular_delta.norm();
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

    double angular_delta_cos = cos(angular_delta_mag / 2.0);
    double angular_delta_sin = sin(angular_delta_mag / 2.0);

    Eigen::Quaterniond dq(angular_delta_cos,
                          angular_delta_sin * angular_delta_dir.x(),
                          angular_delta_sin * angular_delta_dir.y(),
                          angular_delta_sin * angular_delta_dir.z());

    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

    q = q * dq;

    R_prev = pose_.block<3, 3>(0, 0);
    pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    R_curr = pose_.block<3, 3>(0, 0);
}

Eigen::Matrix3d Estimator::skew(const Eigen::Vector3d &angular_delta) {
    Eigen::Matrix3d skew_matrix = Eigen::Matrix3d::Identity();
    skew_matrix(0, 1) = -angular_delta.z();
    skew_matrix(0, 2) = angular_delta.y();
    skew_matrix(1, 2) = -angular_delta.x();
    skew_matrix(1, 0) = angular_delta.z();
    skew_matrix(2, 0) = -angular_delta.y();
    skew_matrix(2, 1) = angular_delta.x();
    return skew_matrix;
}

void Estimator::UpdateRotationMatrix(const Eigen::Vector3d &angular_delta,
                                     Eigen::Matrix3d &R_curr,
                                     Eigen::Matrix3d &R_prev) {
    double angular_delta_mag = angular_delta.norm();
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

    double angular_delta_cos = cos(angular_delta_mag);
    double angular_delta_sin = sin(angular_delta_mag);

    Eigen::Matrix3d dr =
        Eigen::Matrix3d::Identity() +
        angular_delta_sin / angular_delta_mag * (skew(angular_delta_dir)) +
        (1 - angular_delta_cos) / (pow(angular_delta_mag, 2)) *
            (skew(angular_delta_dir) * skew(angular_delta_dir));

    Eigen::Matrix3d rotate_matrix(pose_.block<3, 3>(0, 0));

    rotate_matrix = rotate_matrix * dr;
    R_prev = pose_.block<3, 3>(0, 0);
    pose_.block<3, 3>(0, 0) = rotate_matrix;
    R_curr = pose_.block<3, 3>(0, 0);
}

bool Estimator::GetVelocityDelta(const size_t index_curr,
                                 const size_t index_prev,
                                 const Eigen::Matrix3d &R_curr,
                                 const Eigen::Matrix3d &R_prev, double &delta_t,
                                 Eigen::Vector3d &velocity_delta) {
    if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
        return false;
    }

    const IMUData imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData imu_data_prev = imu_data_buff_.at(index_prev);

    delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d linear_acc_curr =
        GetUnbiasedLinearAcc(imu_data_curr.linear_acceleration, R_curr);

    Eigen::Vector3d linear_acc_prev =
        GetUnbiasedLinearAcc(imu_data_prev.linear_acceleration, R_prev);

    velocity_delta = 0.5 * delta_t * (linear_acc_curr + linear_acc_prev);

    return true;
}

Eigen::Vector3d
Estimator::GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                const Eigen::Matrix3d &R) {
    return R * (linear_acc - linear_acc_bias_) - G_;
}

void Estimator::UpdatePosition(const double &delta_t,
                               const Eigen::Vector3d &velocity_delta) {
    pose_.block<3, 1>(0, 3) += delta_t * vel_ + 0.5 * delta_t * velocity_delta;
    vel_ += velocity_delta;
}

bool Estimator::PublishPose() {
    SavePose(imu_ofs_, pose_);
    message_odom_.header.stamp = ros::Time::now();
    message_odom_.header.frame_id = "inertial";

    message_odom_.child_frame_id = odom_config_.frame_id;

    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    message_odom_.pose.pose.orientation.x = q.x();
    message_odom_.pose.pose.orientation.y = q.y();
    message_odom_.pose.pose.orientation.z = q.z();
    message_odom_.pose.pose.orientation.w = q.w();

    Eigen::Vector3d t = pose_.block<3, 1>(0, 3);
    message_odom_.pose.pose.position.x = t.x();
    message_odom_.pose.pose.position.y = t.y();
    message_odom_.pose.pose.position.z = t.z();

    message_odom_.twist.twist.linear.x = vel_.x();
    message_odom_.twist.twist.linear.y = vel_.y();
    message_odom_.twist.twist.linear.z = vel_.z();

    odom_estimation_pub_.publish(message_odom_);

    return true;
}

void Estimator::Run() {
    while (hasData()) {
        /* code */
        if (UpdatePose()) {
            PublishPose();
        }
    }
}