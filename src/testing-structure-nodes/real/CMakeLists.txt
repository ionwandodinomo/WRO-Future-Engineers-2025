cmake_minimum_required(VERSION 3.8)
project(service_demo_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_py REQUIRED)
rosidl_generate_interfaces(service_demo_interfaces
"srv/SendCommand.srv";
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()