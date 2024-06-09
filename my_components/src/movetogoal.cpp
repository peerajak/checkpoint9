#include "my_components/movetogoal.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
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
using std::placeholders::_1;






namespace my_components
{
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

float degree_to_radian(float degree) { return degree / 180 * pi; }
float radian_to_degree(float rad) { return rad / pi * 180; }


//class MoveToGoal : public rclcpp::Node 

MoveToGoal::MoveToGoal(const rclcpp::NodeOptions& options) : Node("move_to_goal_node") {


    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);


    rclcpp::SubscriptionOptions options2;
    options2.callback_group = callback_group_2;
    subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MoveToGoal::laser_callback, this, std::placeholders::_1),
        options2);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&MoveToGoal::timer1_callback, this),
        callback_group_1);


    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);


  }


void MoveToGoal::timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer move_to_goal Callback ");
    this->move_robot(ling);   
  }

void MoveToGoal::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    float mid_radian = 0.43633333;


        if(is_wall_ahead(msg, mid_radian, this->obstacle)){
            ling.linear.x = 0;
            ling.angular.z = 0;
            nstate = rotate;
            RCLCPP_INFO(this->get_logger(), "WALL detected");
                     rclcpp::shutdown();

        }else{
            ling.linear.x = 0.3;
            ling.angular.z = 0;
            RCLCPP_INFO(this->get_logger(), "Clear road ahead");
        }
  
  }

void MoveToGoal::move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }

bool MoveToGoal::is_wall_ahead(const sensor_msgs::msg::LaserScan::SharedPtr &msg, float mid_radian, float obstacle_thresh){
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




}




#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::MoveToGoal)
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::MoveToGoal)