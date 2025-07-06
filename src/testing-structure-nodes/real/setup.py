from setuptools import setup
package_name = "challenge"
setup(
name=package_name,
version="0.1.0",
packages=[package_name],
data_files=[
("share/ament_index/resource_index/packages",
["resource/" + package_name]),
("share/" + package_name, ["package.xml"]),
],
install_requires=["setuptools"],
zip_safe=True,
maintainer="you",
description="Autonomous RC car ROS 2 package",
entry_points={
"console_scripts": [
        "cameraopen_node = challenge.cameraopen_node:main",
        "navigateopen_node = challenge.navigateopen_node:main",
        "control_node = challenge.control_node:main",
        "cameraobstacle_node = challenge.cameraobstacle_node:main",
        "navigateobstacle_node = challenge.navigateoobstacle_node:main",
        "imu = challenge.imu:main",
        "led_node = challenge.led_node:main",
],
},
)