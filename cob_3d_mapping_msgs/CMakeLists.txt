cmake_minimum_required(VERSION 3.5)
project(cob_3d_mapping_msgs)

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
find_package(cob_object_detection_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(action_files
  "action/PlaneExtraction.action"
  "action/TableObjectCluster.action"
  "action/Trigger.action"
)

set(msg_files
  "msg/CurvedPolygon.msg"
  "msg/CurvedPolygonArray.msg"
  "msg/Feature.msg"
  "msg/FilterObject.msg"
  "msg/Plane.msg"
  "msg/PlaneScene.msg"
  "msg/Point2D.msg"
  "msg/Polygon.msg"
  "msg/PolylinePoint.msg"
  "msg/Shape.msg"
  "msg/ShapeArray.msg"
  "msg/SimilarityScore.msg"
)

set(srv_files
  "srv/GetApproachPoseForPolygon.srv"
  "srv/GetBoundingBoxes.srv"
  "srv/GetGeometryMap.srv"
  "srv/GetObjectsOfClass.srv"
  "srv/GetPlane.srv"
  "srv/GetPointMap.srv"
  "srv/GetTables.srv"
  "srv/ModifyMap.srv"
  "srv/MoveToTable.srv"
  "srv/SetBoundingBoxes.srv"
  "srv/SetGeometryMap.srv"
  "srv/SetPointMap.srv"
  "srv/UpdateMap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${srv_files} ${action_files}
  DEPENDENCIES builtin_interfaces action_msgs actionlib_msgs cob_object_detection_msgs geometry_msgs sensor_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

install(
  FILES cob_3d_mapping_msgs_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)
