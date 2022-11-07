cmake_minimum_required(VERSION 3.5)
project(cob_perception_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ColorDepthImage.msg"
  "msg/ColorDepthImageArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Float64ArrayStamped.msg"
  "msg/Mask.msg"
  "msg/People.msg"
  "msg/Person.msg"
  "msg/PersonStamped.msg"
  "msg/PointCloud2Array.msg"
  "msg/PositionMeasurement.msg"
  "msg/PositionMeasurementArray.msg"
  "msg/Rect.msg"
  "msg/Skeleton.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
