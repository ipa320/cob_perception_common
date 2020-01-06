cmake_minimum_required(VERSION 2.8.3)
project(cob_object_detection_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  std_srvs
)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    AcquireObjectImage.action
    DetectObjects.action
    TrainObject.action
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    Detection.msg
    DetectionArray.msg
    Mask.msg
    MaskArray.msg
    PoseRT.msg
    Rect.msg
    RectArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    AcquireObjectImage.srv
    BaTestEnvironment.srv
    BagTrainObject.srv
    DetectObjects.srv
    SaveRecordedObject.srv
    StartObjectRecording.srv
    StopObjectRecording.srv
    TrainObject.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
    std_srvs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  test/detection_component_testing.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
