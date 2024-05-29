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

#define pi 3.141592654

const float angle_min = -2.3561999797821045;
const float angle_max = 2.3561999797821045;
const float angle_increment = 0.004363333340734243;
const int total_scan_index = 1081;
const int half_scan_index = 540;


float scan_index_to_radian(int scan_index){
  return float(scan_index-half_scan_index)*angle_increment;
}

float scan_index_to_degree(int scan_index){
  return float(scan_index-half_scan_index)*angle_increment/pi*180;
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

    position_reached = false;
  }

private:
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback ");
    this->move_robot(ling);
   
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    float mid_radian = 0.43633333;

    // RCLCPP_INFO(this->get_logger(), "there are %d range
    // values",msg->ranges.size());
    if(position_reached){
    // Use odom to calculate the degree differece from current robot angle, and the desired degree
    }else{
        if(is_wall_ahead(msg, mid_radian, this->obstacle)){
            ling.linear.x = 0;
            ling.angular.z = 0;
            position_reached = true;
            RCLCPP_INFO(this->get_logger(), "WALL detected");
        }else{
            ling.linear.x = 0.5;
            ling.angular.z = 0;
            RCLCPP_INFO(this->get_logger(), "Clear road ahead");
        }
    }
  
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }

  bool is_wall_ahead(const sensor_msgs::msg::LaserScan::SharedPtr &msg, float mid_radian, float obstacle_thresh){
      int number_of_mid_scan_lines = (int)(mid_radian/angle_increment); 
      int begin_mid_scan_index = half_scan_index - int(number_of_mid_scan_lines/2);
      int end_mid_scan_index=  half_scan_index + int(number_of_mid_scan_lines/2);
      float average_range;
      int total_lines=0;
   
     
      for (int i = begin_mid_scan_index ; i< end_mid_scan_index ;i++){
        total_lines++;
        average_range += msg->ranges[i];
      }
      average_range /= total_lines;
         RCLCPP_INFO(this->get_logger(), "begin_mid_scan_index %d end_mid_scan_index %d average_range %f",
          begin_mid_scan_index, end_mid_scan_index, average_range);
     return average_range < obstacle_thresh;// if average_range is less than threshold, then yes! wall ahead.
  }


  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
  float obstacle;
  float degrees;
  bool position_reached;
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