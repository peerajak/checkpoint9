#include "my_components/service_client.hpp"
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


namespace my_components
{
void ServiceClient::timer_callback() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = final_approach;

    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
    timer_->cancel();
  }
void ServiceClient::response_callback(rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto service_response = future.get();
      if (service_response->complete) {
        RCLCPP_INFO(this->get_logger(), "Result: success");
        // lift the cart
        std_msgs::msg::Empty msgs_empty;
        publisher_lift->publish(msgs_empty);
      } else {
        RCLCPP_INFO(this->get_logger(), "Result: failure");
      }

      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

ServiceClient::ServiceClient(const rclcpp::NodeOptions &options) : Node("service_client") {


    client_ = this->create_client<GoToLoading>("approach_shelf");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_callback, this));
    publisher_lift =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 1);
  }

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::ServiceClient)
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::ServiceClient)