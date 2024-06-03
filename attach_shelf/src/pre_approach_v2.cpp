#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
using std::placeholders::_1;

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <geometry_msgs/msg/point.h>
#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/srv/go_to_loading.hpp"

using GoToLoading = custom_interfaces::srv::GoToLoading;
using namespace std::chrono_literals;

#define pi 3.141592654

const float angle_min = -2.3561999797821045;
const float angle_max = 2.3561999797821045;
const float angle_increment = 0.004363333340734243;
const int total_scan_index = 1081;
const int half_scan_index = 540;
enum nodeState { move_to_goal, rotate, approach_shelf, end_program } nstate;
std::string nstate_string[4] = {"move_to_goal", "rotate", "approach_shelf", "end_program"};

float scan_index_to_radian(int scan_index){
  return float(scan_index-half_scan_index)*angle_increment;
}

float scan_index_to_degree(int scan_index){
  return float(scan_index-half_scan_index)*angle_increment/pi*180;
}

float degree_to_radian(float degree) { return degree / 180 * pi; }
float radian_to_degree(float rad) { return rad / pi * 180; }


class MoveToGoal : public rclcpp::Node {
public:
  MoveToGoal(int &argc, char **argv) : Node("move_to_goal_node") {

     message1 = argv[2];
   obstacle = std::stof(message1);
     message2 = argv[3];
     degrees = std::stof(message2);

    RCLCPP_INFO(this->get_logger(), "Got params obstacle: %f, degrees %f",
                obstacle, degrees);

    message3 = argv[4];
    final_approach = message3 == "true";
    std::string msg_info = final_approach== true? message3+" Got params final_approach: true":message3+"Got params final_approach: false";
    RCLCPP_INFO(this->get_logger(),msg_info.c_str());  


    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2;
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MoveToGoal::laser_callback, this, std::placeholders::_1),
        options1);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&MoveToGoal::timer1_callback, this),
        callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);


  }

private:
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer move_to_goal Callback ");
    this->move_robot(ling);
   
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    float mid_radian = 0.43633333;

    // RCLCPP_INFO(this->get_logger(), "there are %d range
    // values",msg->ranges.size());



        if(is_wall_ahead(msg, mid_radian, this->obstacle)){
            ling.linear.x = 0;
            ling.angular.z = 0;
            nstate = rotate;
            RCLCPP_INFO(this->get_logger(), "WALL detected");
        }else{
            ling.linear.x = 0.5;
            ling.angular.z = 0;
            RCLCPP_INFO(this->get_logger(), "Clear road ahead");
        }

  
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }

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

  std::string message1;
  std::string message2;
  std::string message3;
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
  float obstacle;
  float degrees;
  bool position_reached;
  bool final_approach;


};

class Rotation : public rclcpp::Node {
public:
  Rotation(int &argc, char **argv) : Node("rotation_node") {
     message1 = argv[2];
     obstacle = std::stof(message1);
    //this->declare_parameter("degrees", 0.0);
     message2 = argv[3];
     degrees = std::stof(message2);
    RCLCPP_INFO(this->get_logger(), "Got params obstacle: %f, degrees %f",
                obstacle, degrees);
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2 ;
    subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&Rotation::odom_callback, this,
                  std::placeholders::_1),
        options1);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&Rotation::timer1_callback, this),
        callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

   target_yaw_rad_ = degree_to_radian(degrees);
  }

private:
  void execute() {
    rclcpp::Rate loop_rate(1);
    RCLCPP_INFO(this->get_logger(), "My Callback execute() target %f, current %f ",target_yaw_rad_, current_yaw_rad_);
    while (!check_reached_goal_desire_angle() && rclcpp::ok()) {
    //while(true){
    float angular_z_raw =
          radian_difference(target_yaw_rad_, current_yaw_rad_);

      ling.linear.x = 0.0;
      ling.angular.z =
          angular_z_raw < 1.5 ? angular_z_raw : 0.5 * angular_z_raw;
      // if (std::abs(ling.angular.z ) >1)
      //      ling.angular.z *= 0.1;
      move_robot(ling);
      RCLCPP_INFO(
          this->get_logger(),
          "Rotating current pos=['%f','%f'] target rad "
          "'%f',current rad %f, angular speed %f",
          current_pos_.x, current_pos_.y, target_yaw_rad_, current_yaw_rad_,
          ling.angular.z);
      loop_rate.sleep();
    }
    ling.linear.x = 0.0;
    ling.angular.z =0.0;
    move_robot(ling);
    nstate = approach_shelf;
  }

  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer rotation Callback ");
    timer1_->cancel();
    //assert(false);
    std::thread{std::bind(&Rotation::execute, this)}.detach();
   
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_INFO(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }
  double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  bool check_reached_goal_desire_angle(float delta_error = 0.08) {
    float delta_theta =  std::abs(radian_difference(target_yaw_rad_, current_yaw_rad_));
    return delta_theta < delta_error; // IN GOAL return true else false;
  }
    float radian_difference(float first, float second) {
    return std::abs(first - second) <= pi ? first - second
                                            : (first - second) - 2 * pi;
    }
  std::string message1;
  std::string message2;
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
  float obstacle;
  float degrees;
  bool position_reached;
  geometry_msgs::msg::Point desire_pos_, current_pos_;
  geometry_msgs::msg::Quaternion desire_angle_, current_angle_;
  double target_yaw_rad_, current_yaw_rad_;

};


class ServiceClient : public rclcpp::Node {
private:

  bool final_approach;
  std::string message3;
  float obstacle;
  float degrees;
  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback() {
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
  void
  response_callback(rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto service_response = future.get();
      if( service_response->complete){
            RCLCPP_INFO(this->get_logger(), "Result: success");
      }else{
          RCLCPP_INFO(this->get_logger(), "Result: failure");
      }

      nstate = end_program;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  ServiceClient(int &argc, char **argv) : Node("service_client") {

     message3 = argv[4];
    final_approach = message3 == "true";
    std::string msg_info = final_approach== true? "Got params final_approach: true":"Got params final_approach: false";
    RCLCPP_INFO(this->get_logger(),msg_info.c_str());  

    client_ = this->create_client<GoToLoading>("approach_shelf");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_callback, this));
  }

};



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<MoveToGoal> move_to_goal_node =
      std::make_shared<MoveToGoal>(argc, argv);
  std::shared_ptr<Rotation> rotation_node =
      std::make_shared<Rotation>(argc, argv);

  std::shared_ptr<ServiceClient> service_client_node = 
  std::make_shared<ServiceClient>(argc, argv);
  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  nstate = move_to_goal;
  executor.add_node(move_to_goal_node);
  bool work_finish = false;
  while(rclcpp::ok() && !work_finish)  {
      switch(nstate){
          case move_to_goal:
             executor.spin_some();
             if(nstate != move_to_goal){
                 executor.remove_node(move_to_goal_node);                 
                 RCLCPP_INFO(move_to_goal_node->get_logger(), "State Changed to %s",nstate_string[nstate].c_str());
                 executor.add_node(rotation_node);  
             }          
        break;
        case rotate:
             executor.spin_some();
              if(nstate != rotate){
                 executor.remove_node(rotation_node);                 
                 RCLCPP_INFO(move_to_goal_node->get_logger(), "State Changed to %s",nstate_string[nstate].c_str());
                 executor.add_node(service_client_node);            
             }    
        break;
        case approach_shelf: 
             executor.spin_some();
             if(nstate != approach_shelf){
                 //executor.remove_node(service_client_node);                 
                 RCLCPP_INFO(move_to_goal_node->get_logger(), "State Changed to %s",nstate_string[nstate].c_str());          
             }   
        break;
        case  end_program:
                 RCLCPP_INFO(move_to_goal_node->get_logger(), "State Changed to %s",nstate_string[nstate].c_str());
         work_finish = true;  
        break;
        }

  }
  

  rclcpp::shutdown();
  return 0;
}