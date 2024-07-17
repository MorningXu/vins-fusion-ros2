/** =================================================
* @author: MorningXu (morningxu1991@163.com)
* @version v1.0.0
* @date: 2024年06月21日
* @brief: euroc数据集数据发布节点 imu:200Hz cam:20Hz
* @copyright:
* ================================================== */
#include <cstdio>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <boost/algorithm/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <util.hpp>

#include "util.hpp"


class EurocPubNode : public rclcpp::Node {
public:
    EurocPubNode() = default;

    EurocPubNode(const std::string &name): Node(name) {
        RCLCPP_INFO(this->get_logger(), "EuRoC publisher node start!");
        this->declare_parameter("dataset_dir", "/mnt/c/WorkSpace/DataSets/Video/EuRoC/MH_01_easy/mav0");
        this->get_parameter("dataset_dir", dataset_dir_);
        RCLCPP_INFO(this->get_logger(), "Dataset dir is: %s", dataset_dir_.c_str());

        this->declare_parameter("imu_topic", "/imu0");
        this->get_parameter("imu_topic", imu_topic_);
        RCLCPP_INFO(this->get_logger(), "imu_topic dir is: %s", imu_topic_.c_str());

        this->declare_parameter("is_stereo", true);
        this->get_parameter("is_stereo", is_stereo_);
        RCLCPP_INFO(this->get_logger(), "is_stereo is: %b", is_stereo_);

        this->declare_parameter("cam0_topic", "/cam0/image_raw");
        this->get_parameter("cam0_topic", cam0_topic_);
        RCLCPP_INFO(this->get_logger(), "cam0_topic dir is: %s", cam0_topic_.c_str());

        std::thread load_imu_data_thread(&EurocPubNode::LoadImuMsgFromDataSet, this, std::ref(dataset_dir_));
        std::thread load_cam0_thread(&EurocPubNode::LoadCamMsgFromDataset,this, std::ref(dataset_dir_), 0);


        std::thread load_cam1_thread;
        if (is_stereo_) {
            this->declare_parameter("cam1_topic", "/cam1/image_raw");
            this->get_parameter("cam1_topic", cam1_topic_);
            RCLCPP_INFO(this->get_logger(), "cam1_topic dir is: %s", cam1_topic_.c_str());
            load_cam1_thread = std::thread(&EurocPubNode::LoadCamMsgFromDataset,this, std::ref(dataset_dir_), 1);
        }
        load_imu_data_thread.join();
        load_cam0_thread.join();
        if (is_stereo_) {
            load_cam1_thread.join();
        }

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 100);
        cam0_pub_ = this->create_publisher<sensor_msgs::msg::Image>(cam0_topic_, 100);
        cam1_pub_ = this->create_publisher<sensor_msgs::msg::Image>(cam1_topic_, 100);

    }

    void PubData() {
        if (imu_list_.size() < 1 || cam0_list_.size() < 1) {
            if (!is_clear) {
                RCLCPP_INFO(this->get_logger(), "Data Publish Completed ...");
                imu_list_.clear();
                cam0_list_.clear();
                cam1_list_.clear();
                // rclcpp::shutdown();
                is_clear = true;
                return;
            }
            return;
        }
        imu_pub_->publish(imu_list_.front());
        imu_list_.erase(imu_list_.begin());
        if (imu_list_.front().header.stamp.sec == cam0_list_.front().header.stamp.sec && imu_list_.front().header.stamp.nanosec == cam0_list_.front().header.stamp.nanosec) {
            cam0_pub_->publish(cam0_list_.front());
            cam0_list_.erase(cam0_list_.begin());
            if (is_stereo_) {
                cam1_pub_->publish(cam1_list_.front());
                cam1_list_.erase(cam1_list_.begin());
            }
        }
    }

private:
    std::vector<sensor_msgs::msg::Image> cam0_list_;
    std::vector<sensor_msgs::msg::Image> cam1_list_;
    std::vector<sensor_msgs::msg::Imu> imu_list_;
    bool msg_loading_complete_ = false;

    std::string imu_topic_;
    std::string cam0_topic_;
    std::string cam1_topic_;
    std::string dataset_dir_;
    bool is_stereo_;

    bool is_clear = false;

    long imu_data_size_;
    long cam0_data_size_;
    long cam1_data_size_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam0_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam1_pub_;


    /**
     * 从数据集中读取IMU数据
     * @param file_dir imu数据集目录
     * @return 是否成功读取完毕
     */
    void LoadImuMsgFromDataSet(const std::string &file_dir) {
        RCLCPP_INFO(this->get_logger(), "Start Loading Imu data ！！");
        std::fstream imu_csv;
        imu_csv.open(file_dir + "/imu0/data.csv");
        std::string imu_string;
        std::vector<std::string> datastr;
        while (std::getline(imu_csv, imu_string, '\r')) {
            if (imu_string.substr(0, 1) == "#" || imu_string.size() < 3) {
                continue;
            }
            try {
                boost::split(datastr, imu_string, boost::is_any_of(","), boost::token_compress_off);
                sensor_msgs::msg::Imu imu;
                imu.header.frame_id = "imu";
                long stamp = std::stol(datastr[0]);
                imu.header.stamp = rclcpp::Time(stamp);
                imu.orientation.x = 0;
                imu.orientation.y = 0;
                imu.orientation.z = 0;
                imu.orientation.w = 0;
                imu.angular_velocity.x = std::stod(datastr[1]);
                imu.angular_velocity.y = std::stod(datastr[2]);
                imu.angular_velocity.z = std::stod(datastr[3]);
                imu.linear_acceleration.x = std::stod(datastr[4]);
                imu.linear_acceleration.y = std::stod(datastr[5]);
                imu.linear_acceleration.z = std::stod(datastr[6]);
                imu_list_.push_back(imu);
                datastr.clear();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Imu data Load Fail ！！");
                rclcpp::shutdown();
            }
        }
        imu_data_size_ = imu_list_.size();
        RCLCPP_INFO(this->get_logger(), "Imu data Load Completed ！！");
    }

    /**
     * 从数据集读取图像数据
     * @param file_dir cam数据集目录
     * @return 是否成功读取完毕
     */
    void LoadCamMsgFromDataset(const std::string &file_dir, int cam_num = 0) {
        RCLCPP_INFO(this->get_logger(), "Start Loading Cam%d data ！！", cam_num);
        if (cam_num != 0 && cam_num != 1) {
            RCLCPP_ERROR(this->get_logger(), "cam_num must be 0 or 1");
            rclcpp::shutdown();
            return;
        }
        std::fstream cam_csv;
        std::string source_dir = file_dir + "/cam" + std::to_string(cam_num);
        cam_csv.open(source_dir + "/data.csv");
        std::string cam_string;
        std::vector<std::string> datastr;
        while (std::getline(cam_csv, cam_string, '\r')) {
            if (cam_string.substr(0, 1) == "#" || cam_string.size() < 3) {
                continue;
            }
            try {
                boost::split(datastr, cam_string, boost::is_any_of(","), boost::token_compress_off);
                sensor_msgs::msg::Image img;
                std_msgs::msg::Header header;
                header.frame_id = "cam" + std::to_string(cam_num);
                header.stamp = rclcpp::Time(std::stol(datastr[0]));
                std::string img_path = source_dir + "/data/" + datastr[1];
                cv::Mat cvImg = cv::imread(img_path);
                // cv_bridge::CvImage(header, mat_type23encoding(cvImg.type()), cvImg).toImageMsg(img);
                // cv_bridge::CvImage ros_img;
                // ros_img.encoding = mat_type23encoding(cvImg.type());
                // ros_img.header = header;
                // ros_img.image = cvImg;
                // ros_img.toImageMsg(img);
                img.header = header;
                img.height = cvImg.rows;
                img.width = cvImg.cols;
                img.encoding = mat_type23encoding(cvImg.type());
                img.is_bigendian = false;
                img.step = static_cast<sensor_msgs::msg::Image::_step_type>(cvImg.step);
                img.data.assign(cvImg.datastart,cvImg.dataend);

                if (cam_num == 0) {
                    cam0_list_.push_back(img);
                } else {
                    cam1_list_.push_back(img);
                }
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Cam%d data Load Fail ！！", cam_num);
                rclcpp::shutdown();
            }
        }
        if (cam_num == 0) {
            RCLCPP_INFO(this->get_logger(), "Cam%d data list size: %ld", cam_num, cam0_list_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "Cam%d data list size: %ld", cam_num, cam1_list_.size());
        }
        RCLCPP_INFO(this->get_logger(), "Cam%d data Load Completed ！！", cam_num);
    }

    std::string mat_type23encoding(int mat_type) {
        switch (mat_type) {
            case CV_8UC1:
                return "mono8";
            case CV_8UC3:
                return "bgr8";
            case CV_16SC1:
                return "mono16";
            case CV_8UC4:
                return "rgba8";
        }
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto n = std::make_shared<EurocPubNode>("euroc_pub_node");
    rclcpp::WallRate loop_rate(200);
    while (rclcpp::ok()) {
        rclcpp::spin_some(n);
        n->PubData();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
