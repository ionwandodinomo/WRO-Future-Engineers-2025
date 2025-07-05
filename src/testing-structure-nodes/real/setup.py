from setuptools import setup
package_name = "comp"
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
"cameraopen_node = comp.cameraopen_node:main",
"navigateopen_node = comp.navigateopen_node:main",
"control_node = comp.control_node:main",
"cameraobstacle_node = comp.cameraobstacle_node:main",
"navigateobstacle_node = comp.navigateoobstacle_node:main",
"imu = comp.imu:main",
],
},
)