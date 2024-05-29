#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <geometry_msgs/msg/point.h>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;


class SingleThreadTimer : public rclcpp::Node {
public:
  explicit SingleThreadTimer (
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("single_thread_timer_node", options) {



    timer1_ = this->create_wall_timer(
        500ms, std::bind(&SingleThreadTimer::timer1_callback, this));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;



  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback");
  }

  void move_robot(geometry_msgs::msg::Twist &msg) { publisher_->publish(msg); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<SingleThreadTimer> single_thread_timer_node = std::make_shared<SingleThreadTimer>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(single_thread_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}