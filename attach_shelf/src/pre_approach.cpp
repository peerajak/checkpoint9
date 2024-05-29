#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <geometry_msgs/msg/point.h>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

#define pi 3.14

float radian_from_scan_index(int scan_index) {
  float rad;
  if (659 >= scan_index && scan_index >= 495) {
    rad = float(scan_index - 659) / 165 * (pi / 2);
  } else if (164 >= scan_index && scan_index >= 0) {
    rad = float(scan_index) / 165 * (pi / 2);
  }
  return rad;
}

float degree_from_scan_index(int scan_index) {
  float deg;
  if (659 >= scan_index && scan_index >= 495) {
    deg = float(scan_index - 659) / 165 * 90;
  } else if (164 >= scan_index && scan_index >= 0) {
    deg = float(scan_index) / 165 * 90;
  }
  return deg;
}

class Pre_Approach : public rclcpp::Node {
public:
  Pre_Approach() : Node("pre_approach_node") {
    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0.0);
    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<float>();

    degrees = this->get_parameter("degrees").get_parameter_value().get<float>();
    RCLCPP_INFO(this->get_logger(), "Got params obstracle: %f, degrees %f",
                obstacle, degrees);
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2;
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Pre_Approach::laser_callback, this, std::placeholders::_1),
        options1);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&Pre_Approach::timer1_callback, this),
        callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    //this->move_robot(ling);
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback End");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
  
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
  float obstacle;
  float degrees;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<Pre_Approach> laser_timer_node =
      std::make_shared<Pre_Approach>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(laser_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}