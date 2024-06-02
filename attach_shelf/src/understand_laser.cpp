#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tuple>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define pi 3.141592654

const double angle_min = -2.3561999797821045;
const double angle_max = 2.3561999797821045;
const double angle_increment = 0.004363333340734243;
const int total_scan_index = 1081;
const int half_scan_index = 540;

double scan_index_to_radian(int scan_index) {
  return double(scan_index - half_scan_index) * angle_increment;
}

double scan_index_to_degree(int scan_index) {
  return double(scan_index - half_scan_index) * angle_increment / pi * 180;
}

class group_of_laser {
public:
  enum insertable_state { insertable, full } _state;
  int size;
  group_of_laser(double least_intensity)
      : _least_intensity(least_intensity),
        _state(group_of_laser::insertable_state::insertable) {
    size = 0;
  }
  int insert(double a_radian, double a_range, double an_intensity) {
    if (an_intensity > _least_intensity &&
        _state ==
            group_of_laser::insertable_state::insertable) { // insert only if
                                                            // this is true
      if (size == 0) {
        ranges_vector.push_back(a_range);
        radians_vector.push_back(a_radian);
        intensity_vector.push_back(an_intensity);
        size++;
        _state = group_of_laser::insertable_state::insertable;
        return 0;
      } else {
        auto find_it = std::find_if(
            radians_vector.begin(), radians_vector.end(),
            [&a_radian](double r) { return std::abs(r - a_radian) < 0.17; });
        if (find_it < radians_vector.end()) {
          ranges_vector.push_back(a_range);
          radians_vector.push_back(a_radian);
          intensity_vector.push_back(an_intensity);
          size++;
          _state = group_of_laser::insertable_state::insertable;
          return 0;
        } else {
          _state = group_of_laser::insertable_state::full;
          return 1;
        }
      }

    } else {
      return 2;
    }
  }

  std::tuple<double, double>
  get_min_radian_of_the_group_and_corresponding_distance() {
    auto it_min =
        std::min_element(radians_vector.begin(), radians_vector.end());
    double corresponding_distance =
        ranges_vector[std::distance(radians_vector.begin(), it_min)];
    double min_radian = *it_min;
    return std::tuple<double, double>(min_radian, corresponding_distance);
  }
  std::tuple<double, double>
  get_max_radian_of_the_group_and_corresponding_distance() {
    auto it_max =
        std::max_element(radians_vector.begin(), radians_vector.end());
    double corresponding_distance =
        ranges_vector[std::distance(radians_vector.begin(), it_max)];
    double max_radian = *it_max;
    return std::tuple<double, double>(max_radian, corresponding_distance);
  }

private:
  double _least_intensity;
  std::vector<double> ranges_vector;
  std::vector<double> radians_vector;
  std::vector<double> intensity_vector;
};

class UnderstandLaser : public rclcpp::Node {
public:
  UnderstandLaser() : Node("understand_laser_node") {

    //------- 2. Odom related  -----------//
    rclcpp::SubscriptionOptions options2_odom;
    options2_odom.callback_group = callback_group_2_odom;
    subscription_2_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&UnderstandLaser::odom_callback, this, std::placeholders::_1),
        options2_odom);
    tf_static_publisher_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    //------- 3. Laser related  -----------//
    rclcpp::SubscriptionOptions options3_laser;
    options3_laser.callback_group = callback_group_3_laser;
    subscription_3_laser =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&UnderstandLaser::laser_callback, this,
                      std::placeholders::_1),
            options3_laser);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    tf_published = false;
  }

private:
  //------- 2. Odom related  Functions -----------//
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }
  double yaw_theta_from_quaternion(double qx, double qy, double qz, double qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }
  std::tuple<double, double> get_p1_to_p2_perpendicular_vector_laser_coordinate(
      double p1x_laser, double p1y_laser, double p2x_laser, double p2y_laser) {
    double p1_to_p2_laser_x = p2x_laser - p1x_laser;
    double p1_to_p2_laser_y = p2y_laser - p1y_laser;
    double perpen_l1_to_l2_laser_x = 1;
    double perpen_l1_to_l2_laser_y = -p1_to_p2_laser_x / p1_to_p2_laser_y;
    return std::tuple<double, double>{perpen_l1_to_l2_laser_x,
                                      perpen_l1_to_l2_laser_y};
  }
  double yaw_degree_radian_between_perpendicular_and_laser_x(
      double p1p2_perpendicular_x_laser, double p1p2_perpendicular_y_laser) {
    double x_laser = 1.0;
    double y_laser = 0;
    double x_laser_dot_p1p2_perpendicular_laser =
        p1p2_perpendicular_x_laser * x_laser +
        p1p2_perpendicular_y_laser * y_laser;
    double size_of_p1p2_perpendicular_laser =
        std::sqrt(p1p2_perpendicular_x_laser * p1p2_perpendicular_x_laser +
                  p1p2_perpendicular_y_laser * p1p2_perpendicular_y_laser);
    double size_of_x_laser = 1;
    return std::acos(x_laser_dot_p1p2_perpendicular_laser /
                     (size_of_p1p2_perpendicular_laser * size_of_x_laser));
  }

  //------- 3. Laser related Functions -----------//
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (!tf_published) {
      int smallest_allowable_group = 3;
      std::vector<group_of_laser> aggregation_of_groups_of_lasers;
      std::shared_ptr<group_of_laser> gl(new group_of_laser(2000));

      long unsigned int i = 0;
      while (i < msg->ranges.size()) {
        // RCLCPP_INFO(this->get_logger(), " gl state %d", gl->_state);
        while (i < msg->ranges.size() &&
               gl->_state == group_of_laser::insertable_state::insertable) {
          i++;
          // RCLCPP_INFO(this->get_logger(), "working on %ld, gl state %d",
          // i,gl->_state);
          int result_insert = gl->insert(scan_index_to_radian(i),
                                         msg->ranges[i], msg->intensities[i]);
          switch (result_insert) {
          case 0:
            RCLCPP_INFO(this->get_logger(), "inserted %ld, intensity %f", i,
                        msg->intensities[i]);
            break;
          case 1:; // RCLCPP_INFO(this->get_logger(), "full at %ld", i);
            break;
          case 2:; // RCLCPP_INFO(this->get_logger(), "intensity to small %ld,
                   // %f", i,msg->intensities[i]);
            break;
          }
        }
        if (gl->size >= smallest_allowable_group) {
          aggregation_of_groups_of_lasers.push_back(*gl);
        }
        if (gl->_state == group_of_laser::insertable_state::full) {
          RCLCPP_INFO(this->get_logger(), "groups of laser full at %ld", i);
          gl = std::make_shared<group_of_laser>(2000);
          RCLCPP_INFO(this->get_logger(), "new gl created state %d",
                      gl->_state);
        }
      }

      RCLCPP_INFO(this->get_logger(), "There are %ld groups of laser",
                  aggregation_of_groups_of_lasers.size());
      std::tuple<double, double> P1_laser_polar_coordinate =
          aggregation_of_groups_of_lasers.front()
              .get_max_radian_of_the_group_and_corresponding_distance();
      std::tuple<double, double> P2_laser_polar_coordinate =
          aggregation_of_groups_of_lasers.back()
              .get_max_radian_of_the_group_and_corresponding_distance();
      double P1_r_laser_polar_coordinate_aka_b =
          std::get<1>(P1_laser_polar_coordinate);
      double &b = P1_r_laser_polar_coordinate_aka_b;
      double P1_theta_laser_polar_coordinate_aka_theta1 =
          std::get<0>(P1_laser_polar_coordinate);
      double &theta1 = P1_theta_laser_polar_coordinate_aka_theta1;
      double P2_r_laser_polar_coordinate_aka_c =
          std::get<1>(P2_laser_polar_coordinate);
      double &c = P2_r_laser_polar_coordinate_aka_c;
      double P2_theta_laser_polar_coordinate_aka_theta2 =
          std::get<0>(P2_laser_polar_coordinate);
      double &theta2 = P2_theta_laser_polar_coordinate_aka_theta2;
      RCLCPP_INFO(this->get_logger(), "b=%f, theta1= %f, c=%f, theta2=%f", b,
                  theta1, c, theta2);
      // ----------- calculate Pmid_r_laser_coordinate,
      // Pmid_theta_laser_coordinate, Pmid_z=0 in laser coordinate to type
      // tf2::Vector3, aka, point_in_child_coordinates---//

      double P1x_laser_coordinate = b * std::cos(theta1); 
      double P1y_laser_coordinate = b * std::sin(theta1);
      double P2x_laser_coordinate = c * std::cos(theta2);
      double P2y_laser_coordinate = c * std::sin(theta2);
      double Pmidx_laser_coordinate =
          (P1x_laser_coordinate + P2x_laser_coordinate) / 2;
      double Pmidy_laser_coordinate =
          (P1y_laser_coordinate + P2y_laser_coordinate) / 2;
      double Pmidz_laser_coordinate = 0;
      RCLCPP_INFO(this->get_logger(),
                  "Pmidx_laser_coordinate=%f, Pmidy_laser_coordinate= %f",
                  Pmidx_laser_coordinate, Pmidy_laser_coordinate);
      //---- calculate end quotanion in such a way that  the robot is facing mid
      //of the cart----//
      // answer to type tf2::Quaternion
      // - we know P1, P2 in laser coordinate, find a unit vector perpendicular
      // to vector P1-P2.
      std::tuple<double, double> result_p1p2_perpendicular_laser =
          get_p1_to_p2_perpendicular_vector_laser_coordinate(
              P1x_laser_coordinate, P1y_laser_coordinate, P2x_laser_coordinate,
              P2y_laser_coordinate);
      double p1p2_perpendicular_x_laser =
          std::get<0>(result_p1p2_perpendicular_laser);
      double p1p2_perpendicular_y_laser =
          std::get<1>(result_p1p2_perpendicular_laser);

      double cart_roll_laser = 0;
      double cart_pitch_laser = 0;
      double cart_yaw_laser =
          yaw_degree_radian_between_perpendicular_and_laser_x(
              p1p2_perpendicular_x_laser, p1p2_perpendicular_y_laser);
      RCLCPP_INFO(this->get_logger(), "cart_yaw_laser = %f", cart_yaw_laser);
      tf2::Quaternion q_cart_laser;
      q_cart_laser.setRPY(cart_roll_laser, cart_pitch_laser, cart_yaw_laser);

      // --------- TF2 Calculation of Laser Position w.r.t robot_odom Coordinate
      // ------------//

      geometry_msgs::msg::TransformStamped tf_laser_to_odom;
      rclcpp::Time now = this->get_clock()->now();
      std::string fromFrame = "robot_odom";
      std::string toFrame = "robot_front_laser_link";
      try {
        tf_laser_to_odom =
            tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    toFrame.c_str(), fromFrame.c_str(), ex.what());
        return;
      }

      tf2::Quaternion q(tf_laser_to_odom.transform.rotation.x,
                        tf_laser_to_odom.transform.rotation.y,
                        tf_laser_to_odom.transform.rotation.z,
                        tf_laser_to_odom.transform.rotation.w);
      tf2::Vector3 p(tf_laser_to_odom.transform.translation.x,
                     tf_laser_to_odom.transform.translation.y,
                     tf_laser_to_odom.transform.translation.z);
      tf2::Transform transform(q, p);
      // swithc x with y because our math model's x is gazebo's y and vice versa
      tf2::Vector3 point_in_laser_coordinates(0,//P1x_laser_coordinate,
                                              0,//P1y_laser_coordinate,
                                              0);
      tf2::Vector3 point_in_odom_coordinates =
          transform * point_in_laser_coordinates;
      tf2::Quaternion q_cart_robotodom = transform * q_cart_laser;

      //------------ broadcast TF cart to robot_odom

      std::string fromFrameRel = "cart_frame";
      std::string toFrameRel = "robot_front_laser_link";//"robot_odom";
      geometry_msgs::msg::TransformStamped trans;
      rclcpp::Time now2 = this->get_clock()->now();
      trans.header.stamp = now2;
      trans.header.frame_id = fromFrameRel;
      trans.child_frame_id = toFrameRel;
      trans.transform.translation.x = point_in_odom_coordinates.getX();
      trans.transform.translation.y = point_in_odom_coordinates.getY();
      trans.transform.translation.z = point_in_odom_coordinates.getZ();
      trans.transform.rotation.x = q_cart_robotodom.getX();
      trans.transform.rotation.y = q_cart_robotodom.getY();
      trans.transform.rotation.z = q_cart_robotodom.getZ();
      trans.transform.rotation.w = q_cart_robotodom.getW();

      tf_static_publisher_->sendTransform(trans);

      tf_published = true;
    }
  }

  bool is_wall_ahead(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                     double mid_radian, double obstacle_thresh) {
    int number_of_mid_scan_lines = (int)(mid_radian / angle_increment);
    int begin_mid_scan_index =
        half_scan_index - int(number_of_mid_scan_lines / 2);
    int end_mid_scan_index =
        half_scan_index + int(number_of_mid_scan_lines / 2);
    double average_range;
    int total_lines = 0;

    for (int i = begin_mid_scan_index; i < end_mid_scan_index; i++) {
      total_lines++;
      average_range += msg->ranges[i];
    }
    average_range /= total_lines;
    RCLCPP_INFO(
        this->get_logger(),
        "begin_mid_scan_index %d end_mid_scan_index %d average_range %f",
        begin_mid_scan_index, end_mid_scan_index, average_range);
    return average_range < obstacle_thresh; // if average_range is less than
                                            // threshold, then yes! wall ahead.
  }

  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  bool tf_published;
  //------- 2. Odom related  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_2_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  double current_yaw_rad_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher_;
  //------- 3. Laser related  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_laser;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscription_3_laser;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  // double sleep_time1 = 1.0;

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