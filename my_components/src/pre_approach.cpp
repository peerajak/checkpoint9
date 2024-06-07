#include "my_components/pre_approach.hpp"
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
enum nodeState { move_to_goal, rotate, move_under_shelf, load_shelf } nstate;

float scan_index_to_radian(int scan_index){
  return float(scan_index-half_scan_index)*angle_increment;
}

float scan_index_to_degree(int scan_index){
  return float(scan_index-half_scan_index)*angle_increment/pi*180;
}

float degree_to_radian(float degree) { return degree / 180 * pi; }
float radian_to_degree(float rad) { return rad / pi * 180; }


//class PreApproach : public rclcpp::Node 

PreApproach::PreApproach(const rclcpp::NodeOptions& options) : Node("move_to_goal_node") {


    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    
    rclcpp::SubscriptionOptions options3;    
    options3.callback_group = callback_group_3;
    subscription3_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproach::odom_callback, this,
                  std::placeholders::_1),
        options3);
  
   target_yaw_rad_ = degree_to_radian(degrees);


    rclcpp::SubscriptionOptions options2;
    options2.callback_group = callback_group_2;
    subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&PreApproach::laser_callback, this, std::placeholders::_1),
        options2);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer1_callback, this),
        callback_group_1);
    position_reached = false;

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);


  }


void PreApproach::timer1_callback() {
    if(!position_reached){
    RCLCPP_INFO(this->get_logger(), "Timer move_to_goal Callback ");
    this->move_robot(ling);
    }else{
    RCLCPP_INFO(this->get_logger(), "Timer rotation Callback ");
    timer1_->cancel();
    //assert(false);
    std::thread{std::bind(&PreApproach::execute, this)}.detach();
    }

   
  }

void PreApproach::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    float mid_radian = 0.43633333;

    // RCLCPP_INFO(this->get_logger(), "there are %d range
    // values",msg->ranges.size());



        if(is_wall_ahead(msg, mid_radian, this->obstacle)){
            ling.linear.x = 0;
            ling.angular.z = 0;
            nstate = rotate;
            RCLCPP_INFO(this->get_logger(), "WALL detected");
            position_reached = true;
        }else{
            ling.linear.x = 0.5;
            ling.angular.z = 0;
            RCLCPP_INFO(this->get_logger(), "Clear road ahead");
        }

  
  }
void PreApproach::move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }

bool PreApproach::is_wall_ahead(const sensor_msgs::msg::LaserScan::SharedPtr &msg, float mid_radian, float obstacle_thresh){
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



void PreApproach::execute() {
    rclcpp::Rate loop_rate(1);
    RCLCPP_INFO(this->get_logger(), "My Callback execute() target %f, current %f ",target_yaw_rad_, current_yaw_rad_);
    while (!check_reached_goal_desire_angle() && rclcpp::ok()) {
    //while(true){
    float angular_z_raw =
          radian_difference(target_yaw_rad_, current_yaw_rad_);

      ling.linear.x = 0.0;
      ling.angular.z =
          angular_z_raw < 1.5 ? angular_z_raw : 0.5 * angular_z_raw;
      // if (std::abs(ling.angular.z ) >1)
      //      ling.angular.z *= 0.1;
      move_robot(ling);
      RCLCPP_INFO(
          this->get_logger(),
          "Rotating current pos=['%f','%f'] target rad "
          "'%f',current rad %f, angular speed %f",
          current_pos_.x, current_pos_.y, target_yaw_rad_, current_yaw_rad_,
          ling.angular.z);
      loop_rate.sleep();
    }
    ling.linear.x = 0.0;
    ling.angular.z =0.0;
    move_robot(ling);
    nstate = move_under_shelf;
  }

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_INFO(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }
double PreApproach::yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }

bool PreApproach::check_reached_goal_desire_angle(float delta_error) {
    float delta_theta =  std::abs(radian_difference(target_yaw_rad_, current_yaw_rad_));
    return delta_theta < delta_error; // IN GOAL return true else false;
  }
float PreApproach::radian_difference(float first, float second) {
    return std::abs(first - second) <= pi ? first - second
                                            : (first - second) - 2 * pi;
  }



}




#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)