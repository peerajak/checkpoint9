#ifndef COMPOSITION__MOVETOGOAL_COMPONENT_HPP_
#define COMPOSITION__MOVETOGOAL_COMPONENT_HPP_

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

class MoveToGoal : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  MoveToGoal(const rclcpp::NodeOptions &options);

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

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;

  float obstacle = 0.3;
  float degrees = -90.0;

};

} // namespace my_components
#endif // COMPOSITION__MOVETOGOAL_COMPONENT_HPP_