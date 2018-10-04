cmake_minimum_required(VERSION 2.8.3)
project(ipa_3d_fov_visualization)


find_package(catkin REQUIRED COMPONENTS
            cv_bridge
            image_geometry
            roscpp
            std_msgs
            sensor_msgs
            visualization_msgs
            )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ipa_3d_fov_visualization src/ipa_3d_fov_visualization.cpp)


target_link_libraries(ipa_3d_fov_visualization ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})



