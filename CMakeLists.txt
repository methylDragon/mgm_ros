cmake_minimum_required(VERSION 2.8.3)
project(mgm_ros)
set(CMAKE_CXX_FLAGS "-O3 -DNDEBUG -ffast-math -march=native")

find_package(catkin REQUIRED COMPONENTS
  mgm
  roscpp
  image_transport
  message_filters
  image_geometry
  image_proc
  sensor_msgs
  stereo_msgs
  cv_bridge
  dynamic_reconfigure
  nodelet
)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/Config.cfg
)

catkin_package(
  CATKIN_DEPENDS mgm image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/MGMNodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
