cmake_minimum_required(VERSION 3.0.2)
project(cob_perception_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES 
    ColorDepthImage.msg
    ColorDepthImageArray.msg
    Detection.msg
    DetectionArray.msg
    Float64ArrayStamped.msg
    Mask.msg
    People.msg
    Person.msg
    PersonStamped.msg
    PointCloud2Array.msg
    PositionMeasurement.msg 
    PositionMeasurementArray.msg
    Rect.msg
    Skeleton.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
	geometry_msgs
	sensor_msgs
	std_msgs
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
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
