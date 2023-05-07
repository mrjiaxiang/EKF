#include "eskf_kitti/eskf/ros_wapper.h"
#include <glog/logging.h>

using namespace eskf_kitti;

ROSWapper::ROSWapper(ros::NodeHandle &nh, Json::Value &value) {
    nh_ = nh;
    imu_topic_ = value["imu_topic"].asString();
    gps_topic_ = value["gps_topic"].asString();

    LOG(INFO) << "imu topic : " << imu_topic_;
    LOG(INFO) << "gps topic : " << gps_topic_;

    std::vector<double> state_std_vec;
    const Json::Value array = value["init_state_std"];
    for (int i = 0; i < array.size(); i++) {
        state_std_vec.push_back(array[i].asDouble());
    }
    state_std_.setZero();
    // state_std_.diagonal() =
    //     Eigen::Map<Eigen::Matrix<double, 15, 1>>(state_std_vec.data());
    noise_a_ = value["noise_a"].asDouble();
    noise_g_ = value["noise_g"].asDouble();
    noise_ba_ = value["noise_ba"].asDouble();
    noise_bg_ = value["noise_bg"].asDouble();

    v_x_ = value["v_x"].asDouble();
    v_y_ = value["v_y"].asDouble();
    v_z_ = value["v_z"].asDouble();

    const Eigen::Vector3d I_p_Gps(0., 0., 0.);

    imu_gps_process_ptr_ = std::make_shared<IMUGPSProcess>(
        noise_a_, noise_g_, noise_ba_, noise_bg_, I_p_Gps);

    registerSubPub();
}

ROSWapper::~ROSWapper() {}

void ROSWapper::LogState(const State &state) { return; }

void ROSWapper::LogGps(const GPSData gps_data) { return; }

void ROSWapper::ConvertStateToRosTopic(const State &state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.p[0];
    pose.pose.position.y = state.p[1];
    pose.pose.position.z = state.p[2];

    // const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = state.q.x();
    pose.pose.orientation.y = state.q.y();
    pose.pose.orientation.z = state.q.z();
    pose.pose.orientation.w = state.q.w();

    ros_path_.poses.push_back(pose);
}

void ROSWapper::registerSubPub() {
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 1000,
                                               &ROSWapper::imuCallback, this);
    sub_gps_ = nh_.subscribe<sensor_msgs::NavSatFix>(
        gps_topic_, 100, &ROSWapper::gpsCallback, this);

    state_pub_ = nh_.advertise<nav_msgs::Path>("fused_path", 10);
}

void ROSWapper::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    IMUData imu_data;
    imu_data.stamp = imu_msg->header.stamp.toSec();
    imu_data.acc << imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
    imu_data.gyro << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z;

    State fused_state;
    const bool ok =
        imu_gps_process_ptr_->ProcessImuData(imu_data, &fused_state);
    if (!ok) {
        return;
    }

    ConvertStateToRosTopic(fused_state);

    state_pub_.publish(ros_path_);
}
void ROSWapper::gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg) {
    // if (gps_msg->status.status != 2) {
    //     LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
    //     return;
    // }
    GPSData gps_data;
    gps_data.stamp = gps_msg->header.stamp.toSec();
    gps_data.lla << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude;
    gps_data.cov =
        Eigen::Map<const Eigen::Matrix3d>(gps_msg->position_covariance.data());
    imu_gps_process_ptr_->ProcessGpsPositionData(gps_data);
}