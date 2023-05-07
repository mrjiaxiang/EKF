#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <deque>
#include <glog/logging.h>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "eskf_kitti/eskf/imu_gps_process.h"

namespace eskf_kitti {
class ROSWapper {
  public:
    ROSWapper(ros::NodeHandle &nh, Json::Value &value);
    ~ROSWapper();

    void registerSubPub();

    void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg);

    void LogState(const State &state);
    void LogGps(const GPSData gps_data);

    void ConvertStateToRosTopic(const State &state);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_gps_;

    ros::Publisher state_pub_;
    nav_msgs::Path ros_path_;

    std::shared_ptr<IMUGPSProcess> imu_gps_process_ptr_;

    std::string imu_topic_;
    std::string gps_topic_;

    Eigen::Matrix<double, 15, 15> state_std_; // 当前状态的方差阵

    float noise_a_;
    float noise_g_;
    float noise_ba_;
    float noise_bg_;

    // variables for V .Observation noise
    float v_x_;
    float v_y_;
    float v_z_;
};

} // namespace eskf_kitti
