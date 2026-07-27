from glob import glob

from setuptools import setup

package_name = "dynamic_obstacle_benchmarks"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workspace User",
    maintainer_email="user@example.com",
    description="Dynamic obstacle avoidance benchmarks for ROS 2 simulation stacks.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "person_avoidance_benchmark = dynamic_obstacle_benchmarks.person_avoidance_benchmark:main",
        ],
    },
)
