cmake_minimum_required(VERSION 3.0.2)
project(cob_object_detection_visualizer)

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
	${catkin_BUILD_PACKAGES}
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
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
	${catkin_INCLUDE_DIRS}
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
