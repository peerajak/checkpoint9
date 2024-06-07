#ifndef COMPOSITION__SERVER_COMPONENT_HPP_
#define COMPOSITION__SERVER_COMPONENT_HPP_


#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cassert>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>

using namespace std::chrono_literals;

using GoToLoading = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

#define pi 3.14
namespace my_components {




const double angle_min = -2.3561999797821045;
const double angle_max = 2.3561999797821045;
const double angle_increment = 0.004363333340734243;
const int total_scan_index = 1081;
const int half_scan_index = 540;


double scan_index_to_radian(int scan_index);
double scan_index_to_degree(int scan_index);
double radian_difference(double first, double second); 
double magnitude_of_vector(double x_length, double y_length);

class group_of_laser {
public:
  enum insertable_state { insertable, full } _state;
  int size;
  group_of_laser(double least_intensity);
  int insert(double a_radian, double a_range, double an_intensity); 
  std::tuple<double, double>  get_min_radian_of_the_group_and_corresponding_distance();
  std::tuple<double, double>  get_max_radian_of_the_group_and_corresponding_distance();

private:
  double _least_intensity;
  std::vector<double> ranges_vector;
  std::vector<double> radians_vector;
  std::vector<double> intensity_vector;
};


class MidLegsTFService : public rclcpp::Node {
public:
  MidLegsTFService(const rclcpp::NodeOptions&);
private:
  //------- 0. internal use --------------//
  enum serviceState {
    state_zero,
    service_activated,
    tf_already_published,
    approach_shelf,
    approach_shelf2,
    service_completed_success,
    service_completed_failure,
    service_deactivated
  } nstate;
 std::string nstate_string[8]
      = { "state_zero", "service_activated", "tf_already_published", "approach_shelf" ,"approach_shelf2",
      "service_completed_success","service_completed_failure","service_deactivated"};



  bool attach_to_shelf;

  //------- 1. timer_1 related -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_1_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1_twist;
  rclcpp::TimerBase::SharedPtr timer1_;
  geometry_msgs::msg::Twist ling;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_move_robot1{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_move_robot1;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_move_robot2{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_move_robot2;

  //------- 2. Odom related  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_2_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  double current_yaw_rad_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher_;
  double obstacle = 0.3;
  bool tf_published;
  tf2::Vector3 k_point_in_odom_coordinates;
  //------- 3. Laser related  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_laser;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscription_3_laser;

  //--------4. Service related -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_4_service;
  rclcpp::Service<GoToLoading>::SharedPtr srv_4_service;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_4_static_broadcaster_;

  //--------5. load/ unload related ----------//
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_5_load;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_5_unload;

  //--------- Private Methods --------------------//
  //------- 1. timer_1 related Functions -----------//
  void timer1_callback();
  void move_robot(geometry_msgs::msg::Twist &msg);
  //------- 2. Odom related  Functions -----------//
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double yaw_theta_from_quaternion(double qx, double qy, double qz, double qw);
  std::tuple<double, double> get_p1_to_p2_perpendicular_vector_laser_coordinate(
      double p1x_laser, double p1y_laser, double p2x_laser, double p2y_laser);
  double yaw_degree_radian_between_perpendicular_and_laser_x(
      double p1p2_perpendicular_x_laser, double p1p2_perpendicular_y_laser);
  std::tuple<double, double> get_obstacle_cm_into_obstacle(
      double p1_to_p2_perpendicular_vector_x_laser_coordinate,
      double p1_to_p2_perpendicular_vector_y_laser_coordinate,
      double pmid_x_laser_coordinate, double pmid_y_laser_coordinate);
  //------- 3. Laser related Functions -----------//
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  //--------4. Service related Functions-----------//
  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response); 
};

 


} //namespace

#endif // COMPOSITION__SERVER_COMPONENT_HPP_