cmake_minimum_required(VERSION 2.8.3)
project(ipa_3d_fov_visualization)


find_package(catkin REQUIRED COMPONENTS
            cv_bridge
            image_geometry
            roscpp
            sensor_msgs
            visualization_msgs
            )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ipa_3d_fov_visualization src/ipa_3d_fov_visualization.cpp)

add_dependencies(ipa_3d_fov_visualization ${catkin_EXPORTED_TARGETS})

target_link_libraries(ipa_3d_fov_visualization ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ipa_3d_fov_visualization
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
