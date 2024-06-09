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

def generate_launch_description():
    """Generate launch description with multiple components."""
    #LogInfo(msg=(EnvironmentVariable(name='USER'),
    #                        'starting')),


    container3 = ComposableNodeContainer(
            name='my_container3',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachClient',
                    name='service_client'),
                ComposableNode(
                    package='my_components',
                    plugin='my_components::MidLegsTFService',
                    name='server_component'),
            ],
            output='screen',
    )


    return launch.LaunchDescription([container3 ])