#pragma once
#include <iostream>
#include <string>

struct IMUConfig {
    /* data */
    std::string device_name;
    std::string frame_id;
    std::string topic_name;

    // gravity constant:
    struct {
        double x;
        double y;
        double z;
    } gravity;

    // bias:
    struct {
        struct {
            double x;
            double y;
            double z;
        } angular_velocity;
        struct {
            double x;
            double y;
            double z;
        } linear_acceleration;
    } bias;

    // angular velocity noises:
    double gyro_bias_stddev;
    double gyro_noise_stddev;

    // linear acceleration noises:
    double acc_bias_stddev;
    double acc_noise_stddev;
};

struct OdomConfig {
    /* data */
    // general info:
    std::string frame_id;
    struct {
        std::string ground_truth;
        std::string estimation;
    } topic_name;
};
