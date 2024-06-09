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
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::MoveToGoal',
                    name='movetogoal'),
            ],
            output='screen',
    )

    container2 = ComposableNodeContainer(
            name='my_container2',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::Rotation',
                    name='rotation'),
            ],
            output='screen',
    )

    container3 = ComposableNodeContainer(
            name='my_container3',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachClient',
                    name='service_client')
            ],
            output='screen',
    )

    server_node = Node(
        package='my_components',
        executable='AttachServerNode',
        output='screen',
        emulate_tty=True
        )

    return launch.LaunchDescription([container1, 
            RegisterEventHandler(
            OnProcessExit(
                target_action=container1,
                on_exit=[
                    LogInfo(msg=('event handler start container2')),
                    container2
                ]
            )
        ), 
            RegisterEventHandler(
            OnProcessExit(
                target_action=container2,
                on_exit=[
                    LogInfo(msg=('event handler start container3, and server')),
                    server_node,
                    container3
                ]
            )
        ),   
    ])