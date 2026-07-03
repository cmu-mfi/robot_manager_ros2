cmake_minimum_required(VERSION 3.8)
project(robot_manager_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# List the files
set(action_files
  "action/JointGoal.action"
  "action/PoseGoal.action"
)
set(service_files
  "srv/Home.srv"
  "srv/Park.srv"
  "srv/SetPayload.srv"
  "srv/SetIo.srv"
)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${service_files}
  DEPENDENCIES action_msgs std_msgs geometry_msgs
)

ament_package()
