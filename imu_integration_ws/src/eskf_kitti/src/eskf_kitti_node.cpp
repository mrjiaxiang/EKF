#include "eskf_kitti/eskf/imu_gps_process.h"
#include "eskf_kitti/eskf/ros_wapper.h"
#include "eskf_kitti/global_defination/global_defination.h"

#include <ros/ros.h>

#include <fstream>

#include <glog/logging.h>
#include <jsoncpp/json/json.h>

using namespace eskf_kitti;

int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";

    LOG(INFO) << "FLAGS_log_dir : " << FLAGS_log_dir;

    ros::init(argc, argv, "eskf_kitti_node");
    ros::NodeHandle nh;

    std::string config_path = WORK_SPACE_PATH + "/config/ins_eskf.json";
    std::ifstream infile(config_path);
    if (!infile.is_open()) {
        exit - 1;
    }
    Json::Reader reader;
    Json::Value root;
    reader.parse(infile, root);

    LOG(INFO) << " eskf start";
    ROSWapper ros_wapper(nh, root);
    std::cout << "start ." << std::endl;

    ros::spin();
    return 0;
}