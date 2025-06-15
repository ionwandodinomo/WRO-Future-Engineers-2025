from setuptools import setup
package_name = "open"
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
"camera_node = open.camera_node:main",
"navigate_node = open.navigate_node:main",
"control_node = open.control_node:main",
"imu = open.imu:main",
],
},
)