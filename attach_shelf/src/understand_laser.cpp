#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;



#define pi 3.141592654

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

class band {
private:
  float _min_allow_range_value;
  float _max_allow_range_value;
  float _min_allow_intensity_value;
  int _size;
  std::vector<float> msg_radian;
  std::vector<float> msg_range_values;


public:
  enum insertable_state { insertable, full } _state;

  band(float min_allow_range, float max_allow_range, float min_allow_intensity)
      : _min_allow_range_value(min_allow_range), _max_allow_range_value(max_allow_range),
       _min_allow_intensity_value(min_allow_intensity), _size(0),
        _state(band::insertable_state::insertable) {}

  int insert(float a_radian, float range_value, float intensity_value) {
    float interested_value = std::min(range_value, _max_allow_range_value);
    //if intensity is not strong enough, it is not the baricate
    float _only_strong_intensity_min_allow_range_value = intensity_value > _min_allow_intensity_value?
    _min_allow_range_value: 0;
    if (interested_value > _only_strong_intensity_min_allow_range_value) {
      if (msg_radian.size() == 0) { // firstly pushed
        msg_radian.push_back(a_radian);
        msg_range_values.push_back(interested_value);
        _size++;
        _state = band::insertable_state::insertable;
        return 0;
      }
      // check consecutivte index
      auto find_it = std::find_if(
          msg_radian.begin(), msg_radian.end(),
          [&a_radian](float r) { return std::abs(r - a_radian) < 0.1; });
      if (find_it < msg_radian.end()) {
        msg_radian.push_back(a_radian);
        msg_range_values.push_back(interested_value);
        _size++;
        _state = band::insertable_state::insertable;
        return 0;
      } else {
        _state = band::insertable_state::full;
        return 1;
      }

    } else {
      return 2;
    }
  }

  std::tuple<float, float> get_boundary() { // return min,max

    if (msg_radian.size() > 1)
      return std::tuple<float, float>(msg_radian[0], msg_radian.back());
    if (msg_radian.size() == 1)
      return std::tuple<float, float>(msg_radian[0], msg_radian[0]);

    return std::tuple<float, float>(0, 0);
  }

  float get_deepest_value() {
    auto it_max = std::max_element(msg_range_values.begin(), msg_range_values.end());
    return *it_max;
  }
  std::tuple<float, float> get_deepest_radian() {
    auto it_max = std::max_element(msg_range_values.begin(), msg_range_values.end());
    int deepest_radian = msg_radian[std::distance(msg_range_values.begin(), it_max)];

    return std::tuple<float, float>(deepest_radian, *it_max);
  }
  std::tuple<float, float> get_shallowest_radian() {
    auto it_min = std::min_element(msg_range_values.begin(), msg_range_values.end());
    int shallowest_radian =
        msg_radian[std::distance(msg_range_values.begin(), it_min)];

    return std::tuple<float, float>(shallowest_radian, *it_min);
  }

  std::tuple<float, float> get_broadest_mid_radian() {
    float broadest_radian = (msg_radian[0] + msg_radian.back()) / 2;
    /*auto find_it = std::find_if(msg_radian.begin(), msg_radian.end(),
                                [&broadest_radian](float r) {
                                  return std::abs(r - broadest_radian) < 0.1;
                                });*/
    return std::tuple<float, float>(broadest_radian, get_statistic_min());
  }

  int get_size() { return _size; }
  unsigned int get_msg_radian_size() { return msg_radian.size(); }
  unsigned int get_msg_range_values_size() { return msg_range_values.size(); }

  float get_statistic_max() {
    return *std::max_element(msg_range_values.begin(), msg_range_values.end());
  }
  float get_statistic_min() {
    return *std::min_element(msg_range_values.begin(), msg_range_values.end());
  }
  float get_statistic_mean() {
    return float(std::accumulate(msg_range_values.begin(), msg_range_values.end(), 0)) /
           _size;
  }
};


class UnderstandLaser : public rclcpp::Node {
public:
  UnderstandLaser() : Node("understand_laser_node") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2;
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&UnderstandLaser::laser_callback, this,
                  std::placeholders::_1),
        options1);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&UnderstandLaser::timer1_callback, this),
        callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback ");
    this->move_robot(ling);
  
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    /*
    float mid_radian = 0.43633333;

    // RCLCPP_INFO(this->get_logger(), "there are %d range
    // values",msg->ranges.size());
    if(is_wall_ahead(msg, mid_radian, 0.9)){
        ling.angular.z = 0;
         RCLCPP_INFO(this->get_logger(), "WALL detected");
    }else{
        ling.angular.z = 0.5;
    }

    std::vector<band> aggregation_of_bands;
    std::vector<std::tuple<float, int>> front_ranges;
    for (int i = 495; i < 660; i++) {
      // if (msg->ranges[i] < 200 && msg->ranges[i] > wide_band_min) {
      front_ranges.push_back(std::tuple<float, int>(msg->ranges[i], i));
      // }
    }
    for (int i = 0; i < 165; i++) {
      // if (msg->ranges[i] < 200 && msg->ranges[i] > wide_band_min) {
      front_ranges.push_back(std::tuple<float, int>(msg->ranges[i], i));
      //}
    }

    auto ita = front_ranges.begin();
    std::shared_ptr<band> b_band(new band(tolerated_min, interested_max));

    while (ita < front_ranges.end()) {

      while (ita < front_ranges.end() &&
             b_band->_state == band::insertable_state::insertable) {
        float inserting_radian = radian_from_scan_index(std::get<1>(*ita));
        RCLCPP_DEBUG(this->get_logger(), "inserting radian %f:value %f",
                     inserting_radian, std::get<0>(*ita));
        int insert_result = b_band->insert(inserting_radian, std::get<0>(*ita));
        if (insert_result == 0) {
          RCLCPP_DEBUG(this->get_logger(), "inserted");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "NOT inserted %d", insert_result);
        }
        ita++;
      }

      std::tuple<float, float> boundary_aband = b_band->get_boundary();
      RCLCPP_DEBUG(this->get_logger(),
                   "inserted a band (%f,%f) size%d with max:value %f",
                   std::get<0>(boundary_aband), std::get<1>(boundary_aband),
                   b_band->get_size(), b_band->get_statistic_max());

      if (b_band->get_size() >= smallest_allowable_band) {
        aggregation_of_bands.push_back(*b_band);
      }

      if (b_band->_state == band::insertable_state::full) {
        b_band = std::make_shared<band>(tolerated_min, interested_max);
      }
    }*/
  }

  bool is_wall_ahead(const sensor_msgs::msg::LaserScan::SharedPtr &msg, float mid_radian, float obstacle_thresh){
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


  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  // float sleep_time1 = 1.0;

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