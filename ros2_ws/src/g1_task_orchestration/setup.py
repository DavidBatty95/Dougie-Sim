from glob import glob

from setuptools import setup


package_name = "g1_task_orchestration"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Workspace User",
    maintainer_email="user@example.com",
    description="Safe mission-level sequencing for G1 object transport.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "g1_transport_server = g1_task_orchestration.transport_server:main",
        ],
    },
)
