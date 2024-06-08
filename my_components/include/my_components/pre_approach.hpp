#ifndef COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION__PRE_APPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
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
using std::placeholders::_1;

#define pi 3.141592654
namespace my_components {

float scan_index_to_radian(int scan_index);
float scan_index_to_degree(int scan_index);
float degree_to_radian(float degree);
float radian_to_degree(float rad);

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  PreApproach(const rclcpp::NodeOptions &options);

private:
  enum nodeState { move_to_goal, rotate, move_under_shelf, load_shelf } nstate;
  void timer1_callback();
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void move_robot(geometry_msgs::msg::Twist &msg);
  bool is_wall_ahead(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                     float mid_radian, float obstacle_thresh);

  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription3_;
  float obstacle = 0.5;
  float degrees = -90.0;

  void execute();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw);
  bool check_reached_goal_desire_angle(float delta_error = 0.08);
  float radian_difference(float first, float second);
  bool position_reached;
  geometry_msgs::msg::Point desire_pos_, current_pos_;
  geometry_msgs::msg::Quaternion desire_angle_, current_angle_;
  double target_yaw_rad_, current_yaw_rad_;
};

} // namespace my_components
#endif // COMPOSITION__PRE_APPROACH_COMPONENT_HPP_