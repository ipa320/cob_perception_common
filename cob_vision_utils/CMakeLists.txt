cmake_minimum_required(VERSION 3.0.2)
project(cob_vision_utils)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    common/include
    ros/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    visualization_msgs
  DEPENDS
    OpenCV
)


###########
## Build ##
###########

include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  common/src/VisionUtils.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -D__COB_ROS__")
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


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


