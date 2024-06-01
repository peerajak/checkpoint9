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
#include <vector>

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


  
class group_of_laser{
  public:
  enum insertable_state { insertable, full } _state;
  int size;
  group_of_laser(float least_intensity):_least_intensity(least_intensity),_state( group_of_laser::insertable_state::insertable){size=0; }
  int insert(float a_radian, float a_range, float an_intensity){
    if(an_intensity > _least_intensity && _state ==  group_of_laser::insertable_state::insertable){//insert only if this is true
      if(size == 0){
        ranges_vector.push_back(a_range);
        radians_vector.push_back(a_radian);
        intensity_vector.push_back(an_intensity);
        size++;
        _state =  group_of_laser::insertable_state::insertable;
        return 0;
      }else{
        auto find_it = std::find_if(
          radians_vector.begin(), radians_vector.end(),
          [&a_radian](float r) { return std::abs(r - a_radian) < 0.17; });
         if(find_it < radians_vector.end()){
            ranges_vector.push_back(a_range);
            radians_vector.push_back(a_radian);
            intensity_vector.push_back(an_intensity);
            size++;
            _state =  group_of_laser::insertable_state::insertable;
            return 0;
         }else {
        _state =  group_of_laser::insertable_state::full;
        return 1;
         }
      }
     
    } else {
      return 2;
    }
  }
  
  

  std::tuple<float, float> get_min_radian_of_the_group_and_corresponding_distance(){
      auto it_min = std::min_element(radians_vector.begin(), radians_vector.end());
      float corresponding_distance =
        ranges_vector[std::distance(radians_vector.begin(), it_min)];
      float min_radian = *it_min;
      return std::tuple<float, float>(min_radian, corresponding_distance);
  }
  std::tuple<float, float> get_max_radian_of_the_group_and_corresponding_distance(){
      auto it_max = std::max_element(radians_vector.begin(), radians_vector.end());
      float corresponding_distance =
        ranges_vector[std::distance(radians_vector.begin(), it_max)];
      float max_radian = *it_max;
      return std::tuple<float, float>(max_radian, corresponding_distance);
  }
  private:
  
  float _least_intensity;
  std::vector<float> ranges_vector;
  std::vector<float> radians_vector;
  std::vector<float> intensity_vector;

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

    tf_published = false;
  }

private:
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback ");
    //this->move_robot(ling);
  
  }
  

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    
    if(!tf_published){    
     int smallest_allowable_group = 3;
     std::vector<group_of_laser> aggregation_of_groups_of_lasers;
     std::shared_ptr<group_of_laser> gl(new group_of_laser(2000));

     long unsigned int i =0;
     while(i< msg->ranges.size()){
     //RCLCPP_INFO(this->get_logger(), " gl state %d", gl->_state); 
     while(gl->_state == group_of_laser::insertable_state::insertable){
        i++;
         //RCLCPP_INFO(this->get_logger(), "working on %ld, gl state %d", i,gl->_state);       
           int result_insert = gl->insert(scan_index_to_radian(i),
            msg->ranges[i], msg->intensities[i]);
            switch(result_insert){
            case 0:
             RCLCPP_INFO(this->get_logger(), "inserted %ld, intensity %f", i,msg->intensities[i]);
             break;
            case 1:
             ;//RCLCPP_INFO(this->get_logger(), "full at %ld", i);
             break;
            case 2:
             ;//RCLCPP_INFO(this->get_logger(), "intensity to small %ld, %f", i,msg->intensities[i]);
             break;
            }
        }
        if(gl->size >= smallest_allowable_group){
                    aggregation_of_groups_of_lasers.push_back(*gl);
        }
        if(gl->_state == group_of_laser::insertable_state::full){
                RCLCPP_INFO(this->get_logger(), "groups of laser full at %ld", i);
               gl = std::make_shared<group_of_laser>(2000);
               RCLCPP_INFO(this->get_logger(), "new gl created state %d", gl->_state);  
        }
     }
     
     
     RCLCPP_INFO(this->get_logger(), "There are %ld groups of laser", aggregation_of_groups_of_lasers.size());
     tf_published = true;
    }
   
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
  bool tf_published;
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