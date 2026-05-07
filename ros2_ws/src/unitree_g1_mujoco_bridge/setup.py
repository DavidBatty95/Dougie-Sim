from setuptools import setup

package_name = "unitree_g1_mujoco_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workspace User",
    maintainer_email="user@example.com",
    description="Bridge Unitree G1 Mujoco state and commands into ROS 2 control topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "g1_mujoco_bridge = unitree_g1_mujoco_bridge.g1_mujoco_bridge:main",
            "g1_cmd_vel_teleop = unitree_g1_mujoco_bridge.g1_cmd_vel_teleop:main",
            "g1_hold_pose_controller = unitree_g1_mujoco_bridge.g1_hold_pose_controller:main",
            "g1_policy_cmd_vel_controller = unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller:main",
            "g1_clamp_button = unitree_g1_mujoco_bridge.g1_clamp_button:main",
            "g1_sim_sensors = unitree_g1_mujoco_bridge.g1_sim_sensors:main",
        ],
    },
)
