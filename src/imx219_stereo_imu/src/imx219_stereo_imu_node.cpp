/** =================================================
* @author: MorningXu (morningxu1991@163.com)
* @version v1.0.0
* @date: 2024年07月03日
* @brief:
* @copyright:
* ================================================== */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "imx219_stereo_imu/ICM20948.h"

class Imx219StereoImuNode : public rclcpp::Node {
public:
  Imx219StereoImuNode() = default;

  Imx219StereoImuNode(const std::string &name): Node(name) {
    RCLCPP_INFO(this->get_logger(), "%s is start!", this->get_name());
    this->declare_parameter("imu_topic","/imu0");
    this->get_parameter("imu_topic",imu_topic_);
    imuInit(&enMotionSensorType);
    if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType) {
      RCLCPP_INFO(this->get_logger(), "Motion sersor is ICM-20948\n");
    } else {
      RCLCPP_INFO(this->get_logger(), "Motion sersor NULL\n");
    }

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_,2000);
  }

  void PubImuData() {
    imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
    auto imu = std::make_unique<sensor_msgs::msg::Imu>();
    imu->header.frame_id="";
    imu->header.stamp = this->get_clock()->now();
    tempq.setRPY(stAngles.fRoll,stAngles.fPitch,stAngles.fYaw);
    imu->orientation.x = tempq.getX();
    imu->orientation.y = tempq.getY();
    imu->orientation.z = tempq.getZ();
    imu->orientation.w = tempq.getW();
    // double[9] orientation_covarian   ce
    imu->orientation_covariance = { 0,0,0,0,0,0,0,0,0 };
    //geometry_msgs/msg/Vector3 angular_velocity
    imu->angular_velocity.x = stGyroRawData.fX;
    imu->angular_velocity.y = stGyroRawData.fY;
    imu->angular_velocity.z = stGyroRawData.fZ;
    //double[9] angular_velocity_covariance
    imu->angular_velocity_covariance = { 0,0,0,0,0,0,0,0,0 };
    //geometry_msgs/msg/Vector3 linear_acceleration
    imu->linear_acceleration.x = stAccelRawData.fX;
    imu->linear_acceleration.y = stAccelRawData.fY;
    imu->linear_acceleration.z = stAccelRawData.fZ;
    // double[9] linear_acceleration_covariance
    imu->linear_acceleration_covariance = { 0,0,0,0,0,0,0,0,0 };
    imu_publisher_->publish(*imu);
  }

private:
  IMU_EN_SENSOR_TYPE enMotionSensorType;
  IMU_ST_ANGLES_DATA stAngles; // 角度
  IMU_ST_SENSOR_DATA stGyroRawData; // 角加速度
  IMU_ST_SENSOR_DATA stAccelRawData; // 线加速度
  IMU_ST_SENSOR_DATA stMagnRawData; // 磁力计

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  std::string imu_topic_;

  tf2::Quaternion tempq;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  auto n = std::make_shared<Imx219StereoImuNode>("imx219_stereo_imu_node");
  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {
    rclcpp::spin_some(n);
    n->PubImuData();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
