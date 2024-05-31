#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include <cassert>
#include <memory>
#include <algorithm>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

using GoToLoading = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;
#define pi 3.14
/* This is a service server 
     attach_to_shelf (true/false): 
If True, 
  - the service will do the final approach. This is, it will publish the cart_frame transform, 
  - it will move the robot underneath the shelf and it will lift it. 
If False,
  - the robot will only publish the cart_frame transform, but it will not do the final approach.
          
     complete (true/false): 
- False if the laser only detects 1 shelf leg or none. 
- True if the final approach is successful.

 */

 /*
     Input 
- odom subscriber
- laser subscriber
- /approach_shelf service server

    Internal Calculation lib
- TF2

    Output
- Topic client /elevator_up and /elevator_down
- geometric twist publisher to robot/cmd_vel
- TF2 publisher
 */

/*
Callback group
1. timer callback: to move robot in constant speed
2. odom callback
3. laser callback
4. service callback
*/



class MidLegsTFService : public rclcpp::Node {
public:
  MidLegsTFService() : Node("mid_legs_tf_service_node") {
   //------ 0. internal members ----------------//
   nstate = service_deactivated;
   //------ callback group together -------------//
   
   callback_group_1_timer = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
   callback_group_2_odom= this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
   callback_group_3_laser= this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
   callback_group_4_service= this->create_callback_group(
       rclcpp::CallbackGroupType::MutuallyExclusive);

   //------- 1. timer_1 related -----------//  
    timer1_ = this->create_wall_timer(
        100ms, std::bind(&MidLegsTFService::timer1_callback, this), callback_group_1_timer);
   publisher_1_twist =  this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  //------- 2. Odom related  -----------//
    rclcpp::SubscriptionOptions options2_odom;
    options2_odom.callback_group = callback_group_2_odom;
    subscription_2_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MidLegsTFService::odom_callback, this,
                  std::placeholders::_1),
        options2_odom);

  //------- 3. Laser related  -----------//
    rclcpp::SubscriptionOptions options3_laser;
    options3_laser.callback_group = callback_group_3_laser;
    subscription_3_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MidLegsTFService::laser_callback, this, std::placeholders::_1),
        options3_laser);

  //--------4. Service Server related -----------//
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    //rclcpp::SubscriptionOptions options4_service;
    //options4_service.callback_group = callback_group_4_service;
    srv_4_service= create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&MidLegsTFService::service_callback, this, _1, _2),
        qos_profile.get_rmw_qos_profile() , 
        callback_group_4_service);
    tf_4_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  
 //--------5. load/ unload related ----------//
   publisher_5_load = this->create_publisher<std_msgs::msg::Empty>("elevator_up", 10);  
   publisher_5_unload = this->create_publisher<std_msgs::msg::Empty>("elevator_down", 10);
  }

private:
  //------- 0. internal use --------------//
  //_service_activated is
  // if completed, or not started, delete those bands, and TFs. Because future clients may use this service on different scenarioes
  // if service is called for the first time by a new client.
  enum serviceState { service_activated, tf_published, approach_shelf,
   service_completed_success,service_completed_failure, service_deactivated } nstate;


 /*     attach_to_shelf (true/false): 
If True, 
  - the service will do the final approach. This is, it will publish the cart_frame transform, 
  - it will move the robot underneath the shelf and it will lift it. 
If False,
  - the robot will only publish the cart_frame transform, but it will not do the final approach.
  */        
  bool attach_to_shelf;


  //------- 1. timer_1 related -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_1_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1_twist;
  rclcpp::TimerBase::SharedPtr timer1_;
  geometry_msgs::msg::Twist ling;

  //------- 2. Odom related  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_2_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  double current_yaw_rad_;

  //------- 3. Laser related  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_laser;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_3_laser;

  //--------4. Service related -----------//
  rclcpp::CallbackGroup::SharedPtr  callback_group_4_service;
  rclcpp::Service<GoToLoading>::SharedPtr srv_4_service;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_4_static_broadcaster_;

  //--------5. load/ unload related ----------//
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_5_load;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_5_unload;


  //--------- Private Methods --------------------//
  //------- 1. timer_1 related Functions -----------//
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
        if(nstate == approach_shelf){
            this->move_robot(ling);     
         }
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher_1_twist->publish(msg); }
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
  double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }
  //------- 3. Laser related Functions -----------//
 void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      switch (nstate) {
      case service_activated:
      //1. get current position from odom_callback
      //2. perform laser measurement, find the band between two legs
      //3. calculate the position( in world space) of middle of the two leg and statically tf publish it.
      //4. set nstate to tf_published
      nstate = tf_published;
      break;
      case tf_published:
        //perform switching state base on bool attach_to_shelf 
        if(attach_to_shelf){
          nstate = approach_shelf;
        }else{
        nstate = service_deactivated;
        }
      break;
      case approach_shelf:
      //move to under shelf using tf
      //only set ling. no publish to cmd_vel
      // once the robot is in desired position, set nstate to service_completed success/failure
      break;
      case service_completed_success:
      // notify service callback to send respond
      // set nstate to service_deactivated
      break;
      case service_completed_failure:
      // notify service callback to send respond
      // set nstate to service_deactivated
      break;
      case service_deactivated:
      //perform remove bands, and TFs for future use.
      break;
      }
        RCLCPP_INFO(this->get_logger(), "Laser Callback Start");


    
  }
 //--------4. Service related Functions-----------//
  void
  service_callback(const std::shared_ptr<GoToLoading::Request> request,
                   const std::shared_ptr<GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service Callback");
    nstate = service_activated;
       //         request->laser_data.header.frame_id.c_str());
      response->complete = true;
      //_service_activated = false;
  }


};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto attach_shelf_service_server = std::make_shared<MidLegsTFService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(attach_shelf_service_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}