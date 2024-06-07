#ifndef COMPOSITION__SERVER_COMPONENT_HPP_
#define COMPOSITION__SERVER_COMPONENT_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include <std_msgs/msg/empty.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <geometry_msgs/msg/point.h>
#include <rclcpp/rclcpp.hpp>
using std::placeholders::_1;
using GoToLoading = custom_interfaces::srv::GoToLoading;
using namespace std::chrono_literals;

#define pi 3.141592654
namespace my_components {



} // namespace my_components

#endif // COMPOSITION__SERVER_COMPONENT_HPP_