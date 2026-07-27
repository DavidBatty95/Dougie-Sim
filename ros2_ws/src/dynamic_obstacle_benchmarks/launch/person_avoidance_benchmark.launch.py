from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = LaunchConfiguration("config")
    mode = LaunchConfiguration("mode")
    backend = LaunchConfiguration("backend")
    seed = LaunchConfiguration("seed")
    people = LaunchConfiguration("people")
    goals = LaunchConfiguration("goals")
    output = LaunchConfiguration("output")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution([FindPackageShare("dynamic_obstacle_benchmarks"), "config", "person_avoidance_benchmark.yaml"]),
            ),
            DeclareLaunchArgument("mode", default_value="smoke"),
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("seed", default_value="1"),
            DeclareLaunchArgument("people", default_value="1"),
            DeclareLaunchArgument("goals", default_value="2"),
            DeclareLaunchArgument("output", default_value="results/person_avoidance/launch_smoke.json"),
            Node(
                package="dynamic_obstacle_benchmarks",
                executable="person_avoidance_benchmark",
                name="person_avoidance_benchmark",
                output="screen",
                arguments=[
                    "--config", config,
                    "--mode", mode,
                    "--backend", backend,
                    "--seed", seed,
                    "--people", people,
                    "--goals", goals,
                    "--headless",
                    "--output", output,
                ],
            ),
        ]
    )
