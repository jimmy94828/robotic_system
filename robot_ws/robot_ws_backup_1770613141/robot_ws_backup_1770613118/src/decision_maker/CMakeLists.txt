cmake_minimum_required(VERSION 3.8)
project(decision_maker)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# sinh action GraspPlace
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GraspPlace.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
