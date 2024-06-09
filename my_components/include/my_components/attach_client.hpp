#ifndef COMPOSITION__SERVICE_CLIENT_HPP_
#define COMPOSITION__SERVICE_CLIENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <geometry_msgs/msg/point.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using std::placeholders::_1;
using GoToLoading = custom_interfaces::srv::GoToLoading;
using namespace std::chrono_literals;

#define pi 3.141592654
namespace my_components {

class AttachClient : public rclcpp::Node {
private:
  bool final_approach = true;
  float obstacle = 0.4;
  float degrees = -90.0;
  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_lift;

  void timer_callback();
  void response_callback(rclcpp::Client<GoToLoading>::SharedFuture future);

public:
  AttachClient(const rclcpp::NodeOptions &options);
};

} // namespace my_components

#endif // COMPOSITION__SERVICE_CLIENT_HPP_