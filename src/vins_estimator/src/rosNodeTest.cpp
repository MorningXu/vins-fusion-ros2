/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "pcl/point_types.h"

Estimator estimator;

std::queue<sensor_msgs::msg::Imu> imu_buf;
std::queue<sensor_msgs::msg::PointCloud> feature_buf;
std::queue<sensor_msgs::msg::Image> img0_buf;
std::queue<sensor_msgs::msg::Image> img1_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::msg::Image &img_msg) {
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::msg::Image &img_msg) {
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::msg::Image &img_msg) {
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg.encoding == "8UC1") {
        sensor_msgs::msg::Image img;
        img.header = img_msg.header;
        img.height = img_msg.height;
        img.width = img_msg.width;
        img.is_bigendian = img_msg.is_bigendian;
        img.step = img_msg.step;
        img.data = img_msg.data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process() {
    while (1) {
        if (STEREO) {
            cv::Mat image0, image1;
            // std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty()) {
                double time0 = img0_buf.front().header.stamp.sec + img0_buf.front().header.stamp.nanosec * 1e-9;
                double time1 = img1_buf.front().header.stamp.sec + img1_buf.front().header.stamp.nanosec * 1e-9;
                // 0.003s sync tolerance
                if (time0 < time1 - 0.003) {
                    img0_buf.pop();
                    printf("throw img0\n");
                } else if (time0 > time1 + 0.003) {
                    img1_buf.pop();
                    printf("throw img1\n");
                } else {
                    time = img0_buf.front().header.stamp.sec + img0_buf.front().header.stamp.nanosec * 1e-9;
                    // header = img0_buf.front().header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if (!image0.empty())
                estimator.inputImage(time, image0, image1);
        } else {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty()) {
                time = img0_buf.front().header.stamp.sec + img0_buf.front().header.stamp.nanosec * 1e-9;
                header = img0_buf.front().header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if (!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::msg::Imu &imu_msg) {
    double t = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
    double dx = imu_msg.linear_acceleration.x;
    double dy = imu_msg.linear_acceleration.y;
    double dz = imu_msg.linear_acceleration.z;
    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;
    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::msg::PointCloud &feature_msg) {
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg.points.size(); i++) {
        int feature_id = feature_msg.channels[0].values[i];
        int camera_id = feature_msg.channels[1].values[i];
        double x = feature_msg.points[i].x;
        double y = feature_msg.points[i].y;
        double z = feature_msg.points[i].z;
        double p_u = feature_msg.channels[2].values[i];
        double p_v = feature_msg.channels[3].values[i];
        double velocity_x = feature_msg.channels[4].values[i];
        double velocity_y = feature_msg.channels[5].values[i];
        if (feature_msg.channels.size() > 5) {
            double gx = feature_msg.channels[6].values[i];
            double gy = feature_msg.channels[7].values[i];
            double gz = feature_msg.channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    double t = feature_msg.header.stamp.sec + feature_msg.header.stamp.nanosec * 1e-9;
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::msg::Bool &restart_msg) {
    if (restart_msg.data == true) {
        RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::msg::Bool &switch_msg) {
    if (switch_msg.data == true) {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    } else {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::msg::Bool &switch_msg) {
    if (switch_msg.data == true) {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    } else {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("vins_estimator");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc != 2) {
        printf("please intput: rosrun vins vins_node [config file] \n"
            "for example: rosrun vins vins_node "
            "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    std::string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "waiting for image and imu...");

    registerPub(n);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    if (USE_IMU) {
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 2000,
                                                                std::bind(&imu_callback, std::placeholders::_1));
    }
    auto sub_feature = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", 2000,
                                                                            std::bind(&feature_callback,
                                                                                std::placeholders::_1));

    auto sub_img0 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, 200,
                                                                    std::bind(&img0_callback, std::placeholders::_1));
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1;
    if (STEREO) {
        sub_img1 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, 200,
                                                                   std::bind(&img1_callback, std::placeholders::_1));
    }
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins_restart", 100,
                                                                   std::bind(&restart_callback, std::placeholders::_1));
    auto sub_imu_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_imu_switch", 100,
                                                                      std::bind(&imu_switch_callback,
                                                                          std::placeholders::_1));
    auto sub_cam_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_cam_switch", 100,
                                                                      std::bind(&cam_switch_callback,
                                                                          std::placeholders::_1));

    std::thread sync_thread{sync_process};
    rclcpp::spin(n);

    return 0;
}
