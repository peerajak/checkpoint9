import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with multiple components."""
    #LogInfo(msg=(EnvironmentVariable(name='USER'),
    #                        'starting')),
    container1 = ComposableNodeContainer(
            name='pre_approach',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'),
            ],
            output='screen'
    )
   
   # exec ros2 run rclcpp_components component_container
    component_container_manager = ExecuteProcess(
    cmd=[[
        'ros2 run rclcpp_components component_container --ros-args -r __node:=my_container'
    ]],
    shell=True
    )  


    server_node = Node(
        package='my_components',
        name='attach_server',
        executable='AttachServerNode',
        output='screen',
        emulate_tty=True
        )

    return launch.LaunchDescription([#            
            container1,
            component_container_manager,
            server_node
            #RegisterEventHandler(
            #OnProcessExit(
            #    target_action=container1,
            #    on_exit=[
            #        LogInfo(msg=('event handler start server')),
            #        #,container2
            #    ]
            #)
        #) 
    ])