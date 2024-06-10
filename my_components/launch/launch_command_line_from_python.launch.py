import launch
from launch.actions import ExecuteProcess


def generate_launch_description():
    component_container_manager = ExecuteProcess(
    cmd=[[
        'ros2 run rclcpp_components component_container --ros-args -r __node:=my_container'
    ]],
    shell=True
    )  

    return launch.LaunchDescription([component_container_manager])