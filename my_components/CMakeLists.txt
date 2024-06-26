cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(include)

add_library(pre_approach SHARED src/pre_approach.cpp)
target_compile_definitions(pre_approach PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "custom_interfaces"
  "sensor_msgs"
  "tf2" 
  "tf2_ros")
rclcpp_components_register_nodes(pre_approach "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach>\n")

add_library(movetogoal SHARED src/movetogoal.cpp)
target_compile_definitions(movetogoal PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(movetogoal
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "custom_interfaces"
  "sensor_msgs"
  "tf2" 
  "tf2_ros")
rclcpp_components_register_nodes(movetogoal "my_components::MoveToGoal")
set(node_plugins "${node_plugins}my_components::MoveToGoal;$<TARGET_FILE:movetogoal>\n")

add_library(rotation SHARED src/rotation.cpp)
target_compile_definitions(rotation PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(rotation
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "custom_interfaces"
  "sensor_msgs"
  "tf2" 
  "tf2_ros")
rclcpp_components_register_nodes(rotation "my_components::Rotation")
set(node_plugins "${node_plugins}my_components::Rotation;$<TARGET_FILE:movetogoal>\n")

add_library(AttachClient SHARED src/attach_client.cpp)
target_compile_definitions(AttachClient PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(AttachClient
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "custom_interfaces"
  "sensor_msgs"
  "tf2" 
  "tf2_ros")
rclcpp_components_register_nodes(AttachClient "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:movetogoal>\n")



add_library(server_component SHARED src/server_component.cpp)
target_compile_definitions(server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "custom_interfaces"
  "tf2"
  "tf2_ros"
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs")
rclcpp_components_register_nodes(server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:server_component>\n")

add_executable(AttachServerNode
  src/manual_server_composition.cpp)
target_link_libraries(AttachServerNode
server_component
)
ament_target_dependencies(AttachServerNode
rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs custom_interfaces tf2 tf2_ros)



install(TARGETS
  pre_approach
  movetogoal
  rotation
  AttachClient
  server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  AttachServerNode
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
