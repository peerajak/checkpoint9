#include "my_components/server_component.hpp"
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


using namespace std::chrono_literals;

using GoToLoading = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;
#define pi 3.14

namespace my_components {
double scan_index_to_radian(int scan_index) {
  return double(scan_index - half_scan_index) * angle_increment;
}

double scan_index_to_degree(int scan_index) {
  return double(scan_index - half_scan_index) * angle_increment / pi * 180;
}

double radian_difference(double first, double second) {
  return std::abs(first - second) <= pi ? first - second
                                        : (first - second) - 2 * pi;
}

double magnitude_of_vector(double x_length, double y_length){
   return sqrt(x_length*x_length + y_length*y_length);
}

group_of_laser::group_of_laser(double least_intensity)
      : _least_intensity(least_intensity),
        _state(group_of_laser::insertable_state::insertable) {
    size = 0;
  }

int group_of_laser::insert(double a_radian, double a_range, double an_intensity) {
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

std::tuple<double, double> group_of_laser::get_min_radian_of_the_group_and_corresponding_distance() {
    auto it_min =
        std::min_element(radians_vector.begin(), radians_vector.end());
    double corresponding_distance =
        ranges_vector[std::distance(radians_vector.begin(), it_min)];
    double min_radian = *it_min;
    return std::tuple<double, double>(min_radian, corresponding_distance);
  }

std::tuple<double, double> group_of_laser::get_max_radian_of_the_group_and_corresponding_distance() {
    auto it_max =
        std::max_element(radians_vector.begin(), radians_vector.end());
    double corresponding_distance =
        ranges_vector[std::distance(radians_vector.begin(), it_max)];
    double max_radian = *it_max;
    return std::tuple<double, double>(max_radian, corresponding_distance);
  }


AttachServer::AttachServer(const rclcpp::NodeOptions& options) : Node("mid_legs_tf_service_node") {
    //------ 0. internal members ----------------//


 
    tf_published = false;          
    nstate = state_zero;
    //------ callback group together -------------//

    callback_group_1_timer = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3_laser = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_4_service = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    //------- 1. timer_1 related -----------//
    timer1_ = this->create_wall_timer(
        100ms, std::bind(&AttachServer::timer1_callback, this),
        callback_group_1_timer);
    publisher_1_twist =
        this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
    tf_buffer_move_robot1 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_move_robot1 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_move_robot1 );
    tf_buffer_move_robot2 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_move_robot2 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_move_robot2 );


    //------- 2. Odom related  -----------//
    rclcpp::SubscriptionOptions options2_odom;
    options2_odom.callback_group = callback_group_2_odom;
    subscription_2_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&AttachServer::odom_callback, this,
                  std::placeholders::_1),
        options2_odom);
    tf_static_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //------- 3. Laser related  -----------//
    rclcpp::SubscriptionOptions options3_laser;
    options3_laser.callback_group = callback_group_3_laser;
    subscription_3_laser =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&AttachServer::laser_callback, this,
                      std::placeholders::_1),
            options3_laser);

    //--------4. Service Server related -----------//
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    // rclcpp::SubscriptionOptions options4_service;
    // options4_service.callback_group = callback_group_4_service;
    srv_4_service = create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&AttachServer::service_callback, this, _1, _2),
        qos_profile.get_rmw_qos_profile(), callback_group_4_service);
    tf_4_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);




  }

  //--------- AttachServer::Private Methods --------------------//
  //------- 1. timer_1 related Functions -----------//
  void AttachServer::timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    if (nstate == approach_shelf || nstate == approach_shelf2) {    

            this->move_robot(ling);
        RCLCPP_INFO(this->get_logger(), "linear x %f, angular z %f",ling.linear.x,ling.angular.z);

    }
    if( nstate == service_completed_success){
                 ling.linear.x = 0;
            ling.linear.y = 0;
            ling.linear.z = 0;
            ling.angular.x = 0;
            ling.angular.y = 0;
            ling.angular.z = 0;
           this->move_robot(ling);
            RCLCPP_INFO(this->get_logger(), "linear x %f, angular z %f",ling.linear.x,ling.angular.z);
    }

             
  }
  
  void AttachServer::move_robot(geometry_msgs::msg::Twist &msg) {
    publisher_1_twist->publish(msg);
  }



  //------- 2. Odom related  Functions -----------//
  void AttachServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Odom Callback Started");
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }
  
  double AttachServer::yaw_theta_from_quaternion(double qx, double qy, double qz, double qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }

  std::tuple<double, double> AttachServer::get_p1_to_p2_perpendicular_vector_laser_coordinate(
      double p1x_laser, double p1y_laser, double p2x_laser, double p2y_laser) {
    double p1_to_p2_laser_x = p2x_laser - p1x_laser;
    double p1_to_p2_laser_y = p2y_laser - p1y_laser;
    double perpen_l1_to_l2_laser_x = 1;
    double perpen_l1_to_l2_laser_y = -p1_to_p2_laser_x / p1_to_p2_laser_y;
    return std::tuple<double, double>{perpen_l1_to_l2_laser_x,
                                      perpen_l1_to_l2_laser_y};
  }

  double AttachServer::yaw_degree_radian_between_perpendicular_and_laser_x(
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
    double sign_of_anser =
        p1p2_perpendicular_y_laser / abs(p1p2_perpendicular_y_laser);
    return sign_of_anser *
           std::acos(x_laser_dot_p1p2_perpendicular_laser /
                     (size_of_p1p2_perpendicular_laser * size_of_x_laser));
  }

  std::tuple<double, double> AttachServer::get_obstacle_cm_into_obstacle(
      double p1_to_p2_perpendicular_vector_x_laser_coordinate,
      double p1_to_p2_perpendicular_vector_y_laser_coordinate,
      double pmid_x_laser_coordinate, double pmid_y_laser_coordinate) {
    // call the final position point k, or vector k
    double &kcm = this->obstacle; // in meter unit
    double size_of_p1p2_perpendicular_laser =
        std::sqrt(p1_to_p2_perpendicular_vector_x_laser_coordinate *
                      p1_to_p2_perpendicular_vector_x_laser_coordinate +
                  p1_to_p2_perpendicular_vector_y_laser_coordinate *
                      p1_to_p2_perpendicular_vector_y_laser_coordinate);
    double scalar_multiplier = kcm / size_of_p1p2_perpendicular_laser;
    double k_x_laser_coordinate =
        pmid_x_laser_coordinate +
        scalar_multiplier * p1_to_p2_perpendicular_vector_x_laser_coordinate;
    double k_y_laser_coordinate =
        pmid_y_laser_coordinate +
        scalar_multiplier * p1_to_p2_perpendicular_vector_y_laser_coordinate;
    return std::tuple<double, double>{k_x_laser_coordinate,
                                      k_y_laser_coordinate};
  }

  //------- 3. Laser related Functions -----------//
  void AttachServer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Started, nstate %s",nstate_string[nstate].c_str());
    switch (nstate) {
    case service_activated:
    {
      // 1. get current position from odom_callback
      // 2. perform laser measurement, find the band between two legs
      // 3. calculate the position( in world space) of middle of the two leg and
      // statically tf publish it.
      // 4. set nstate to tf_already_published
              RCLCPP_INFO(this->get_logger(), "Laser Callback End, state is %s",nstate_string[nstate].c_str());
       RCLCPP_INFO(this->get_logger(),"laser() service_activated");
      if (!tf_published) {
     RCLCPP_INFO(this->get_logger(),"Publishing TF");
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
        if( aggregation_of_groups_of_lasers.size()<2){
        nstate = service_completed_failure;
        return;
        }
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
        //---- calculate end quotanion in such a way that  the robot is facing
        //mid of the cart----//
        // answer to type tf2::Quaternion
        // - we know P1, P2 in laser coordinate, find a unit vector
        // perpendicular to vector P1-P2.
        std::tuple<double, double> result_p1p2_perpendicular_laser =
            get_p1_to_p2_perpendicular_vector_laser_coordinate(
                P1x_laser_coordinate, P1y_laser_coordinate,
                P2x_laser_coordinate, P2y_laser_coordinate);
        double p1p2_perpendicular_x_laser =
            std::get<0>(result_p1p2_perpendicular_laser);
        double p1p2_perpendicular_y_laser =
            std::get<1>(result_p1p2_perpendicular_laser);

        double cart_roll_laser = 0;
        double cart_pitch_laser = 0;
        double cart_yaw_laser =
            yaw_degree_radian_between_perpendicular_and_laser_x(
                p1p2_perpendicular_x_laser, p1p2_perpendicular_y_laser);
        RCLCPP_INFO(this->get_logger(),
                    "p1p2_x_laser= %f, p1p2_y_laser= %f, cart_yaw_laser = %f",
                    p1p2_perpendicular_x_laser, p1p2_perpendicular_y_laser,
                    cart_yaw_laser);
        tf2::Quaternion q_cart_laser;
        q_cart_laser.setRPY(cart_roll_laser, cart_pitch_laser, cart_yaw_laser);

        // --------- TF2 Calculation of Laser Position w.r.t odom
        // Coordinate
        // ------------//

        geometry_msgs::msg::TransformStamped tf_laser_to_odom;
        rclcpp::Time now = this->get_clock()->now();
        std::string fromFrame = "robot_front_laser_base_link";
        std::string toFrame = "odom";
        try {
          tf_laser_to_odom = tf_buffer_->lookupTransform(toFrame, fromFrame,
                                                         tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                      toFrame.c_str(), fromFrame.c_str(), ex.what());
          nstate = service_completed_failure;
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

        tf2::Vector3 point_in_laser_coordinates(Pmidx_laser_coordinate,
                                                Pmidy_laser_coordinate, 
                                                Pmidz_laser_coordinate);
        tf2::Vector3 point_in_odom_coordinates =
            transform * point_in_laser_coordinates;
        tf2::Quaternion q_cart_robotodom = transform * q_cart_laser;

        //------------ broadcast TF cart to odom

        std::string fromFrameRel = "odom";
        std::string toFrameRel = "cart_frame";
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

        // obstacle frame static TF braodcast
        toFrameRel = "obstacle_frame";
        std::tuple<double, double> k_point_laser =
            get_obstacle_cm_into_obstacle(
                p1p2_perpendicular_x_laser, p1p2_perpendicular_y_laser,
                Pmidx_laser_coordinate, Pmidy_laser_coordinate);
        double k_point_x_laser = std::get<0>(k_point_laser);
        double k_point_y_laser = std::get<1>(k_point_laser);
        tf2::Vector3 k_point_in_laser_coordinates(k_point_x_laser,
                                                  k_point_y_laser, 0);
        k_point_in_odom_coordinates =
            transform * k_point_in_laser_coordinates;
         RCLCPP_INFO(this->get_logger(), "k_point_in_odom_coordinates position in odom cooordinate x: %f, y: %f",
         k_point_in_odom_coordinates.getX(),k_point_in_odom_coordinates.getY());
        geometry_msgs::msg::TransformStamped trans2;
        rclcpp::Time now3 = this->get_clock()->now();
        trans2.header.stamp = now3;
        trans2.header.frame_id = fromFrameRel;
        trans2.child_frame_id = toFrameRel;
        trans2.transform.translation.x = k_point_in_odom_coordinates.getX();
        trans2.transform.translation.y = k_point_in_odom_coordinates.getY();
        trans2.transform.translation.z = k_point_in_odom_coordinates.getZ();
        trans2.transform.rotation.x = q_cart_robotodom.getX();
        trans2.transform.rotation.y = q_cart_robotodom.getY();
        trans2.transform.rotation.z = q_cart_robotodom.getZ();
        trans2.transform.rotation.w = q_cart_robotodom.getW();

        tf_static_publisher_->sendTransform(trans2);
        

        tf_published = true;
        nstate = tf_already_published;
      }
    }
      break;
    case tf_already_published:
            RCLCPP_INFO(this->get_logger(), "Laser Callback End, state is %s",nstate_string[nstate].c_str());
      if (attach_to_shelf) {
        nstate = approach_shelf;
      } else {
        nstate = service_completed_failure;
      }
      break;
    case approach_shelf:
      // move to under shelf using tf
      // only set ling. no publish to cmd_vel
      {
              RCLCPP_INFO(this->get_logger(), "Laser Callback End, state is %s",nstate_string[nstate].c_str());
    std::string fromFrame_tomoveto = "cart_frame";
    std::string toFrame_tofollow = "robot_evelator_base_link";
    geometry_msgs::msg::TransformStamped ttt;
        try {
            rclcpp::Time now = this->get_clock()->now();
            ttt = tf_buffer_move_robot1->lookupTransform(
                toFrame_tofollow, fromFrame_tomoveto,
                tf2::TimePointZero);//
            } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrame_tofollow.c_str(), fromFrame_tomoveto.c_str(), ex.what());
                nstate = service_completed_failure;
            return;
            }

            RCLCPP_INFO(this->get_logger(), "ttt.x: %f, ttt.y: %f, atan radian %f",
            ttt.transform.translation.x,ttt.transform.translation.y, atan(
            ttt.transform.translation.y/ttt.transform.translation.x));  
            double scaleRotationRate = 0.8;
            double scaleForwardSpeed = 1.0;
            ling.linear.x = scaleForwardSpeed * ttt.transform.translation.x;
  
            ling.linear.y = 0;
            if(abs(ttt.transform.translation.x) < 0.02){
               double target_yaw_rad = yaw_theta_from_quaternion(
                ttt.transform.rotation.x, ttt.transform.rotation.y,
                ttt.transform.rotation.z, ttt.transform.rotation.w);
               ling.angular.z = scaleRotationRate*target_yaw_rad;  
               RCLCPP_INFO(this->get_logger(), "target_yaw_rad: %f,",target_yaw_rad);
               if(abs(ttt.transform.translation.x) < 0.01 && target_yaw_rad <0.1){
                  nstate = approach_shelf2;
               }            
           } else {

            ling.angular.x = 0;
            ling.angular.y = 0;
            ling.angular.z = scaleRotationRate * atan(
            ttt.transform.translation.y/ttt.transform.translation.x);
           }


      }
      //  once the robot is in desired position, set nstate to service_completed
      //  success/failure
      break;
    case approach_shelf2:
    {
            RCLCPP_INFO(this->get_logger(), "Laser Callback End, state is %s",nstate_string[nstate].c_str());

            
     std::string fromFrame_tomoveto2 = "obstacle_frame";
    std::string toFrame_tofollow2 = "robot_evelator_base_link";
    geometry_msgs::msg::TransformStamped ttt2;
        try {
            rclcpp::Time now = this->get_clock()->now();
            ttt2 = tf_buffer_move_robot2->lookupTransform(
                toFrame_tofollow2, fromFrame_tomoveto2,
                tf2::TimePointZero);//
            } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrame_tofollow2.c_str(), fromFrame_tomoveto2.c_str(), ex.what());
                nstate = service_completed_failure;
            return;
            }
          
       
            double scaleRotationRate = 0.8;
            double target_yaw_rad = yaw_theta_from_quaternion(
            ttt2.transform.rotation.x, ttt2.transform.rotation.y,
            ttt2.transform.rotation.z, ttt2.transform.rotation.w);
            ling.angular.z = scaleRotationRate*target_yaw_rad;   
            double scaleForwardSpeed = 1.2;
            if(ttt2.transform.translation.x < 0.02) scaleForwardSpeed = 3;
            ling.linear.x = scaleForwardSpeed * ttt2.transform.translation.x;
            ling.linear.y = 0;
            RCLCPP_INFO(this->get_logger(), "ttt2.x: %f, ttt2.y: %f",ttt2.transform.translation.x,ttt2.transform.translation.y);    
               
           RCLCPP_INFO(this->get_logger(), "current pos=['%f','%f'",
                 current_pos_.x, current_pos_.y); 
          RCLCPP_INFO(this->get_logger(), "k_point x: %f, y: %f",
         k_point_in_odom_coordinates.getX(),k_point_in_odom_coordinates.getY());  
         double tolerance_threshold = 0.01;
            if(abs(ttt2.transform.translation.x) < tolerance_threshold){
            ling.linear.x = 0;
            ling.linear.y = 0;
            ling.linear.z = 0;
            ling.angular.x = 0;
            ling.angular.y = 0;
            ling.angular.z = 0;

              nstate = service_completed_success;
           }
       
      }    
     break;
    case service_completed_success:
            RCLCPP_INFO(this->get_logger(), "Laser Callback End, state is %s",nstate_string[nstate].c_str());
     // do nothing to notify service callback to send respond
      break;
    case service_completed_failure:
            RCLCPP_INFO(this->get_logger(), "Laser Callback End, state is %s",nstate_string[nstate].c_str());
     // do nothing to notify service callback to send respond
      break;
    case service_deactivated:
                ling.linear.x = 0;
            ling.linear.y = 0;
            ling.linear.z = 0;
            ling.angular.x = 0;
            ling.angular.y = 0;
            ling.angular.z = 0;
            move_robot(ling);
  rclcpp::shutdown();

      break;
    }


  }

  //--------4. Service related Functions-----------//
  void AttachServer::service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response) {

    nstate = service_activated;
    attach_to_shelf = request->attach_to_shelf;
    if(attach_to_shelf){
            RCLCPP_INFO(this->get_logger(), "Service Callback, state is %d, attach_to_shelf = true",nstate);
    } else{
     RCLCPP_INFO(this->get_logger(), "Service Callback, state is %d, attach_to_shelf = false",nstate);
    }
    rclcpp::Rate rate(5); // ROS Rate at 5Hz
    //         request->laser_data.header.frame_id.c_str());
    while(!(nstate == service_completed_success || nstate == service_completed_failure))
    {
     RCLCPP_INFO(this->get_logger(), "Working on Service nstate %s",nstate_string[nstate].c_str());
     rate.sleep();
    }
    if(nstate == service_completed_success){
        response->complete = true;
         RCLCPP_INFO(this->get_logger(), "service_completed_success",nstate);
        nstate = service_deactivated;

    }else if(nstate == service_completed_failure){
    RCLCPP_INFO(this->get_logger(), "service_completed_failure",nstate);
     response->complete = false;
     nstate = service_deactivated;

    }
   

    //_service_activated = false;
  }


 

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)