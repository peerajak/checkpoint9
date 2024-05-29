#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;



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


class UnderstandLaser : public rclcpp::Node {
public:
  UnderstandLaser() : Node("understand_laser_node") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2;
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&UnderstandLaser::laser_callback, this,
                  std::placeholders::_1),
        options1);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&UnderstandLaser::timer1_callback, this),
        callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
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
    if(is_wall_ahead(msg, mid_radian, 0.9)){
        ling.angular.z = 0;
         RCLCPP_INFO(this->get_logger(), "WALL detected");
    }else{
        ling.angular.z = 0.5;
    }
  }

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


  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  // float sleep_time1 = 1.0;

  std::shared_ptr<UnderstandLaser> laser_timer_node =
      std::make_shared<UnderstandLaser>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(laser_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
/*
header:
  stamp:
    sec: 85
    nanosec: 408000000
  frame_id: robot_front_laser_link
angle_min: -2.3561999797821045
angle_max: 2.3561999797821045
angle_increment: 0.004363333340734243
time_increment: 0.0
scan_time: 0.0
range_min: 0.05999999865889549
range_max: 20.0
ranges:


There are 1081 scan indexes.


angle_min: -2.3561999797821045  (-0.75pi)
angle_max: 2.3561999797821045  (0.75pi)
angle_increment: 0.004363333340734243

Thus there are 4.7124/0.00436333 = 1080 lines for -135 to 135 degree
degree msg->range[0] start from -0.75pi from x axis,


case 1. 0-540 has 541 lines of scan is -0.75pi to 0 radian
radian = (scan_index-540)*angle_increment

case 2 541-1080 has 540 lines of scan is 0 to 0.75 radian
radian = (scan_index-540)*angle_increment

both case can be combined into one case.
radian = (scan_index-540)*angle_increment
*/