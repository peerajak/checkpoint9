# checkpoint9

## Task1 plan
ros2 pkg create --build-type ament_cmake attach_shelf --dependencies tf2_ros geometry_msgs nav_msgs rclcpp custom_interfaces std_msgs sensor_msgs
- Move robot according to parameter given by CLI



## Task2 plan
- Create a service
- Play with laser intensity to detect the two legs of the cart
- try to publish a dynamic TF from a fixed point to the world frame. Show that moving robot does not move the TF
? Do I need to do? - try to publish another dynamic TF from an object to a world frame. Show that moving the object does move the TF.


user:~$ ros2 topic list
/clock
/cmd_vel
/elevator_down
/elevator_up
/odom
/parameter_events
/robot/cmd_vel
/rosout
/scan
/tf
/tf_static
user:~$
