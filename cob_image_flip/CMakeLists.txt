cmake_minimum_required(VERSION 2.8.3)
project(cob_image_flip)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  pcl_ros pcl_conversions)


find_package(catkin REQUIRED COMPONENTS
  cob_vision_utils
  cv_bridge
  image_transport
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS signals)

find_package(OpenCV REQUIRED)

#find_package(PCL REQUIRED)

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
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    kinect_image_flip_nodelet
  CATKIN_DEPENDS
    cob_vision_utils
    cv_bridge
    image_transport
    nodelet
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf
  DEPENDS
    #PCL
    OpenCV
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(kinect_image_flip_nodelet
  ros/src/kinect_image_flip_nodelet.cpp
  ros/src/kinect_image_flip.cpp
)

## Declare a cpp executable
add_executable(kinect_image_flip
  ros/src/kinect_image_flip_main.cpp
  ros/src/kinect_image_flip.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_image_flip_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  #${PCL_LIBRARIES}
)

target_link_libraries(kinect_image_flip
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  #${PCL_LIBRARIES}
)

# set build flags for targets
set_target_properties(kinect_image_flip_nodelet PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(kinect_image_flip PROPERTIES COMPILE_FLAGS -D__LINUX__)



#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS kinect_image_flip_nodelet kinect_image_flip
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY common/include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)
install(DIRECTORY ros/include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY ros/launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
    nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)