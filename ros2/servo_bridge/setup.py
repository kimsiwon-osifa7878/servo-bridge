from glob import glob
from setuptools import find_packages, setup


package_name = "servo_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/servo_bridge"]),
        ("share/servo_bridge", ["package.xml"]),
        ("share/servo_bridge/config", glob("config/*.yaml")),
        ("share/servo_bridge/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial", "PyYAML", "PySide6"],
    zip_safe=True,
    maintainer="osifa",
    maintainer_email="osifa@example.com",
    description="ROS2 serial bridge and desktop UI for ESP32 servos",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = servo_bridge.ros_node:main",
            "control_ui = servo_bridge.ui:main",
        ],
    },
)
