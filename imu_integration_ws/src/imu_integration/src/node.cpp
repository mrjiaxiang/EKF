#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

#include <deque>
#include <glog/logging.h>
#include <iostream>

#include "imu_integration/estimator.hpp"

using namespace std;

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir =
        "/home/melody/example/EKF/imu_integration_ws/src/imu_integration/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "imu_integration_node");

    LOG(INFO) << "start estimator";
    Estimator estimator;
    estimator.Init();
    LOG(INFO) << "init estimator";

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        estimator.Run();

        loop_rate.sleep();
    }

    return 0;
}