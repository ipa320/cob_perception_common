cmake_minimum_required(VERSION 3.0.2)
project(cob_image_flip)

set(catkin_RUN_PACKAGES 
	cob_perception_msgs
	cv_bridge
	geometry_msgs
	image_transport
	nodelet
	pcl_conversions
	pcl_ros
	pluginlib
	roscpp
	sensor_msgs
	stereo_msgs
	tf
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(Boost REQUIRED COMPONENTS)
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
	LIBRARIES
		image_flip_nodelet
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		Boost
		OpenCV
		#PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(image_flip_nodelet
	ros/src/image_flip.cpp
	ros/src/image_flip_nodelet.cpp
)
set_target_properties(image_flip_nodelet PROPERTIES COMPILE_FLAGS -D__LINUX__)
target_link_libraries(image_flip_nodelet
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	#${PCL_LIBRARIES}
)
add_dependencies(image_flip_nodelet ${catkin_EXPORTED_TARGETS})


## Declare a cpp executable
add_executable(image_flip
	ros/src/image_flip.cpp
	ros/src/image_flip_main.cpp
)
set_target_properties(image_flip PROPERTIES COMPILE_FLAGS -D__LINUX__)
target_link_libraries(image_flip
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	#${PCL_LIBRARIES}
)
add_dependencies(image_flip ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS image_flip image_flip_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
