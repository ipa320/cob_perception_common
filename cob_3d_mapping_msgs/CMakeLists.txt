cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_msgs)


find_package(catkin REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  cob_object_detection_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    PlaneExtraction.action
    TableObjectCluster.action
    Trigger.action
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    CurvedPolygonArray.msg
    CurvedPolygon.msg
    Feature.msg
    FilterObject.msg
    Plane.msg
    PlaneScene.msg
    Point2D.msg
    Polygon.msg 
    PolylinePoint.msg
    ShapeArray.msg
    Shape.msg
    SimilarityScore.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    GetApproachPoseForPolygon.srv
    GetBoundingBoxes.srv
    GetGeometryMap.srv
    GetObjectsOfClass.srv
    GetPlane.srv
    GetPointMap.srv
    GetTables.srv
    ModifyMap.srv
    MoveToTable.srv
    SetBoundingBoxes.srv
    SetGeometryMap.srv
    SetPointMap.srv
    UpdateMap.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    cob_object_detection_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    cob_object_detection_msgs
    message_runtime
    geometry_msgs
    sensor_msgs
    std_msgs
)
