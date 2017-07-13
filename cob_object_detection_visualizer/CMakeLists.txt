cmake_minimum_required(VERSION 2.8.3)
project(cob_object_detection_visualizer)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) # RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

# all ROS packages from package.xml (libopencv-dev is system dependency --> sudo apt-get install)
set(catkin_RUN_PACKAGES
  cob_object_detection_msgs
  cv_bridge
  eigen_conversions
  image_transport
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
)

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES} # this makes ${catkin_LIBRARIES} include all libraries of ${catkin_BUILD_PACKAGES}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
INCLUDE_DIRS
	#ros/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	Boost
	OpenCV
	PCL
)
###########
## Build ##
###########

include_directories(
	#ros/include
	${catkin_INCLUDE_DIRS} # same name as find_package
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(object_detection_visualizer ros/src/object_detection_visualizer_node.cpp)
add_dependencies(object_detection_visualizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_detection_visualizer
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS object_detection_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
