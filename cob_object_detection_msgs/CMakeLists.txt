cmake_minimum_required(VERSION 3.5)
project(cob_object_detection_msgs)

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
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(action_files
  "action/AcquireObjectImage.action"
  "action/DetectObjects.action"
  "action/TrainObject.action"
)

set(msg_files
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Mask.msg"
  "msg/MaskArray.msg"
  "msg/PoseRT.msg"
  "msg/Rect.msg"
  "msg/RectArray.msg"
)

set(srv_files
  "srv/AcquireObjectImage.srv"
  "srv/BaTestEnvironment.srv"
  "srv/BagTrainObject.srv"
  "srv/DetectObjects.srv"
  "srv/SaveRecordedObject.srv"
  "srv/StartObjectRecording.srv"
  "srv/StopObjectRecording.srv"
  "srv/TrainObject.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${srv_files} ${action_files}
  DEPENDENCIES builtin_interfaces action_msgs actionlib_msgs geometry_msgs sensor_msgs std_msgs std_srvs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

#catkin_install_python(PROGRAMS
#  test/detection_component_testing.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
