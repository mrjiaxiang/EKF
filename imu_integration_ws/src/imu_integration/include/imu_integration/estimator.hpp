#include <Eigen/Core>
#include <Eigen/Dense>

#include <fstream>
#include <iostream>

#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "imu_integration/config.hpp"
#include "imu_integration/imu_data.hpp"
#include "imu_integration/odom_data.hpp"

class Estimator {
  public:
    Estimator();
    ~Estimator();
    void Init();

    bool hasData();
    bool UpdatePose();
    bool GetAngularDelta(size_t index_curr, size_t index_prev,
                         Eigen::Vector3d &angular_delta);
    Eigen::Vector3d
    GetUnbiasedAngularVel(const Eigen::Vector3d &angular_velocity);

    Eigen::Vector3d GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                         const Eigen::Matrix3d &R);

    void UpdateOrientation(const Eigen::Vector3d &angular_delta,
                           Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev);

    void UpdateRotationMatrix(const Eigen::Vector3d &angular_delta,
                              Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev);

    Eigen::Matrix3d skew(const Eigen::Vector3d &angular_delta);

    bool GetVelocityDelta(const size_t index_curr, const size_t index_prev,
                          const Eigen::Matrix3d &R_curr,
                          const Eigen::Matrix3d &R_prev, double &delta_t,
                          Eigen::Vector3d &velocity_delta);

    void UpdatePosition(const double &delta_t,
                        const Eigen::Vector3d &velocity_delta);

    bool PublishPose();

    void Run();
    inline bool SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ofs << pose(i, j);

                if (i == 2 && j == 3) {
                    ofs << std::endl;
                } else {
                    ofs << " ";
                }
            }
        }

        return true;
    }

  private:
    void imu_callback(const sensor_msgs::ImuConstPtr &msg);
    void odom_callback(const nav_msgs::OdometryConstPtr &msg);

  private:
    ros::NodeHandle private_nh_;

    std::deque<IMUData> imu_data_buff_;
    std::deque<OdomData> odom_data_buff_;

    IMUConfig imu_config_;
    OdomConfig odom_config_;

    ros::Publisher odom_estimation_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;

    bool initialized_ = false;

    // a. gravity constant:
    Eigen::Vector3d G_;
    // b. angular velocity:
    Eigen::Vector3d angular_vel_bias_;
    // c. linear acceleration:
    Eigen::Vector3d linear_acc_bias_;

    // IMU pose estimation:
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();

    nav_msgs::Odometry message_odom_;
    geometry_msgs::PoseStamped this_pose_stamped_;
    nav_msgs::Path path_;
    ros::Publisher path_pub_;

    std::ofstream gt_ofs_;
    std::ofstream imu_ofs_;
};
