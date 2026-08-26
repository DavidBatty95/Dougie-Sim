from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="g1_task_orchestration",
                executable="g1_transport_server",
                name="g1_transport_server",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("g1_task_orchestration"), "config", "transport.yaml"]
                    )
                ],
            )
        ]
    )
