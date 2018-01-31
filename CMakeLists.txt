cmake_minimum_required(VERSION 2.8.3)
project(sgm_stereo)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -mavx -msse4.2 -msse4.1 -msse3 -msse2")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  nodelet
  roscpp
  sensor_msgs
  stereo_msgs
  stereo_image_proc
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sgm_stereo
  CATKIN_DEPENDS cv_bridge image_geometry image_transport message_filters roscpp sensor_msgs stereo_image_proc
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# SGM Lib
add_library(sgm_stereo src/SGMStereo.cpp)
add_dependencies(sgm_stereo ${catkin_EXPORTED_TARGETS})
target_link_libraries(sgm_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS})

# SGM Stereo Nodelet
add_library(${PROJECT_NAME}_nodelet
  src/sgm_stereo_node.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}
)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} sgm_stereo sgm_stereo_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
