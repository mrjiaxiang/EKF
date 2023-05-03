#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <stdlib.h>
#include <streambuf>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

std::vector<std::string> path;

void ReadData(const std::vector<std::string> &path, std::vector<double> &stamps,
              std::vector<Eigen::Vector3d> &accs,
              std::vector<Eigen::Vector3d> &gyros,
              std::vector<Eigen::Vector3d> &gpses,
              std::vector<Eigen::Vector3d> &ref_poses,
              std::vector<Eigen::Quaterniond> &ref_att_quats);

int main(int argc, char **argv) {
    ros::init(argc, argv, "csv_to_bag_node");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open("/home/melody/example/EKF/imu_integration_ws/src/csv_to_bag/data/"
             "sim_imu.bag",
             1U);

    path.push_back("/home/melody/example/EKF/imu_integration_ws/src/"
                   "gnss_ins_sim/data/circle_8/time.csv");
    path.push_back("/home/melody/example/EKF/imu_integration_ws/src/"
                   "gnss_ins_sim/data/circle_8/accel-0.csv");
    path.push_back("/home/melody/example/EKF/imu_integration_ws/src/"
                   "gnss_ins_sim/data/circle_8/gyro-0.csv");
    path.push_back("/home/melody/example/EKF/imu_integration_ws/src/"
                   "gnss_ins_sim/data/circle_8/gps-0.csv");
    path.push_back("/home/melody/example/EKF/imu_integration_ws/src/"
                   "gnss_ins_sim/data/circle_8/ref_pos.csv");
    path.push_back("/home/melody/example/EKF/imu_integration_ws/src/"
                   "gnss_ins_sim/data/circle_8/ref_att_quat.csv");

    std::vector<double> stamps;
    std::vector<Eigen::Vector3d> accs;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> gpses;
    std::vector<Eigen::Vector3d> ref_poses;
    std::vector<Eigen::Quaterniond> ref_att_quats;

    ReadData(path, stamps, accs, gyros, gpses, ref_poses, ref_att_quats);

    for (int i = 0; i < stamps.size(); i++) {
        if (ros::ok()) {
            ROS_DEBUG("the value size = %ld\n", stamps.size());
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time(stamps[i]);
            imu_msg.header.frame_id = "inertial";
            imu_msg.linear_acceleration.x = accs[i].x();
            imu_msg.linear_acceleration.y = accs[i].y();
            imu_msg.linear_acceleration.z = accs[i].z();

            imu_msg.angular_velocity.x = gyros[i].x();
            imu_msg.angular_velocity.y = gyros[i].y();
            imu_msg.angular_velocity.z = gyros[i].z();
            bag.write("/sim/sensor/imu", imu_msg.header.stamp, imu_msg);

            nav_msgs::Odometry odom;
            // odom.child_frame_id = "inertial";
            odom.header.stamp = ros::Time(stamps[i]);
            odom.header.frame_id = "inertial";

            odom.pose.pose.position.x = ref_poses[i].x();
            odom.pose.pose.position.y = ref_poses[i].y();
            odom.pose.pose.position.z = ref_poses[i].z();

            odom.pose.pose.orientation.w = ref_att_quats[i].w();
            odom.pose.pose.orientation.x = ref_att_quats[i].x();
            odom.pose.pose.orientation.y = ref_att_quats[i].y();
            odom.pose.pose.orientation.z = ref_att_quats[i].z();
            bag.write("/pose/ground_truth", odom.header.stamp, odom);
        } else {
            break;
        }
    }
    return EXIT_SUCCESS;
}

// sci 转 double
// 方法 1
double sciToDub(const std::string &str) {
    std::stringstream ss(str);
    double d = 0;
    ss >> d;
    if (ss.fail())

    {
        std::string s = "Unable to format";
        s += str;
        s += "as s number!";
        throw(s);
    }
    return (d);
}

// 方法 2
bool ParseNumber(const char *s, double *d) {
    bool bNegtiveBase, bNegtiveExp; // 分别表示基数和指数是否为负
    int nPreZero = 0;               // 基数前缀0的个数
    const char *p;
    int sum_i = 0;
    double sum_f = 0.0;
    int sum_exp = 0;
    double sum = 0.0;
    bNegtiveBase = bNegtiveExp = false;
    if (!s)
        return false;
    if ('-' == *s) {
        bNegtiveBase = true;
        s++;
    }
    for (; '0' == *s; nPreZero++, s++)
        ; // 统计基数前缀0的个数
    for (; *s != '.' && *s != 'e' && *s != 'E' && *s != '\0'; s++) {
        if (*s < '0' || *s > '9') {
            return false;
        }
        sum_i = sum_i * 10 + *s - '0';
    }
    if (0 == sum_i &&
        0 ==
            nPreZero) // 基数为0且前缀0个数为0，说明是.25、e7、E7、“”之类的数据格式
        return false;
    if ('.' == *s) // 还需要计算小数部分数值
    {
        for (p = s; *p != 'e' && *p != 'E' && *p != '\0'; p++)
            ; // 找到尾数部分的末尾
        if (s == p - 1)
            return false;
        s = p;
        p--;
        for (; *p != '.'; p--) {
            if (*p < '0' || *p > '9')
                return false;
            sum_f = sum_f * 0.1 + 0.1 * (*p - '0');
        }
    }
    if ('e' == *s || 'E' == *s) // 还需要计算阶码
    {
        s++;
        if ('-' == *s) {
            bNegtiveExp = true;
            s++;
        } else if ('+' == *s) {
            bNegtiveExp = false;
            s++;
        }
        nPreZero = 0;
        for (; *s != '\0'; s++) {
            if (*s < '0' || *s > '9') {
                return false;
            }
            sum_exp = sum_exp * 10 + *s - '0';
            nPreZero++;
        }
        if (0 == sum_exp && 0 == nPreZero)
            return false;
    }
    sum = sum_i + sum_f;
    if (bNegtiveExp) // 阶码为负
    {
        while (sum_exp > 0) {
            sum /= 10;
            sum_exp--;
        }
    } else {
        while (sum_exp > 0) {
            sum *= 10;
            sum_exp--;
        }
    }
    if (bNegtiveBase) // 基数为负
        sum = -sum;
    *d = sum;
    return true;
}

void ReadData(const std::vector<std::string> &path, std::vector<double> &stamps,
              std::vector<Eigen::Vector3d> &accs,
              std::vector<Eigen::Vector3d> &gyros,
              std::vector<Eigen::Vector3d> &gpses,
              std::vector<Eigen::Vector3d> &ref_poses,
              std::vector<Eigen::Quaterniond> &ref_att_quats) {

    std::cout << "start readData" << std::endl;
    stamps.clear();
    accs.clear();
    gyros.clear();
    gpses.clear();
    ref_poses.clear();
    ref_att_quats.clear();

    std::vector<std::ifstream> reads;

    for (size_t i = 0; i < path.size(); i++) {
        reads.push_back(std::ifstream(path[i], std::ios::in));
    }

    bool init = false;
    while (true) {
        // 去除标题
        if (!init) {
            init = true;
            for (size_t i = 0; i < reads.size(); i++) {
                std::string strs;
                std::getline(reads[i], strs);
            }
        } else {
            double time;
            {
                std::string strs;
                if (std::getline(reads[0], strs)) {
                    std::cout << "strs = " << strs << std::endl;
                    // time = std::stod(strs);
                    // ParseNumber(strs.c_str(), &time);
                    time = sciToDub(strs) + 0.1;
                    std::cout << "time = " << time << std::endl;
                    // sleep(10000);
                } else {
                    break;
                }
            }

            {
                std::string strs;
                std::string temp;
                strs = "";
                std::getline(reads[1], strs);
                temp = "";
                std::vector<double> acc;
                for (size_t i = 0; i < strs.size(); i++) {
                    if (strs[i] == ',') {
                        double acc_tmp;
                        ParseNumber(temp.c_str(), &acc_tmp);
                        acc.push_back(acc_tmp);
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                double acc_tmp;
                ParseNumber(temp.c_str(), &acc_tmp);
                acc.push_back(acc_tmp);

                strs = "";
                std::getline(reads[2], strs);
                temp = "";
                std::vector<double> gyro;
                for (size_t i = 0; i < strs.size(); i++) {
                    if (strs[i] == ',') {
                        double gyro_tmp;
                        ParseNumber(temp.c_str(), &gyro_tmp);
                        gyro.push_back(gyro_tmp);
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                double gyro_tmp;
                ParseNumber(temp.c_str(), &gyro_tmp);
                gyro.push_back(gyro_tmp);

                strs = "";
                std::getline(reads[3], strs);
                temp = "";
                std::vector<double> gps;
                for (int i = 0; i < strs.size(); ++i) {
                    if (strs[i] == ',') {
                        double gps_tmp;
                        ParseNumber(temp.c_str(), &gps_tmp);
                        gps.push_back(gps_tmp);
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                double gps_tmp;
                ParseNumber(temp.c_str(), &gps_tmp);
                gps.push_back(gps_tmp);

                strs = "";
                std::getline(reads[4], strs);
                temp = "";
                std::vector<double> ref_pos;
                for (int i = 0; i < strs.size(); ++i) {
                    if (strs[i] == ',') {
                        double ref_pos_tmp;
                        ParseNumber(temp.c_str(), &ref_pos_tmp);
                        ref_pos.push_back(ref_pos_tmp);
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                double ref_pos_tmp;
                ParseNumber(temp.c_str(), &ref_pos_tmp);
                ref_pos.push_back(ref_pos_tmp);

                strs = "";
                std::getline(reads[5], strs);
                temp = "";
                std::vector<double> ref_att_quat;
                for (int i = 0; i < strs.size(); ++i) {
                    if (strs[i] == ',') {
                        double ref_att_quat_tmp;
                        ParseNumber(temp.c_str(), &ref_att_quat_tmp);
                        ref_att_quat.push_back(ref_att_quat_tmp);
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                double ref_att_quat_tmp;
                ParseNumber(temp.c_str(), &ref_att_quat_tmp);
                ref_att_quat.push_back(ref_att_quat_tmp);

                stamps.push_back(time);
                accs.push_back(Eigen::Vector3d(acc[0], acc[1], acc[2]));
                gyros.push_back(Eigen::Vector3d(gyro[0], gyro[1], gyro[2]));

                Eigen::Quaterniond q =
                    Eigen::AngleAxisd(90, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(180, Eigen::Vector3d::UnitX());

                q = q.inverse();
                gpses.push_back(Eigen::Vector3d(gps[0], gps[1], gps[2]));

                ref_poses.push_back(
                    q * Eigen::Vector3d(ref_pos[0], ref_pos[1], ref_pos[2]));

                ref_att_quats.push_back(
                    q * Eigen::Quaterniond(ref_att_quat[0], ref_att_quat[1],
                                           ref_att_quat[2], ref_att_quat[3]));
            }
        }
    }
}