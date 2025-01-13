cmake_minimum_required(VERSION 3.5.0)
project(hik_tem)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  camera_info_manager
  std_msgs
  rospy
  message_interface
)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDES})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDES})


catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp std_msgs rospy message_interface)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_library(hik_tem src/hik_camera.cpp)
target_link_libraries(hik_tem
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    hcnetsdk PlayCtrl
)

add_dependencies(hik_tem 
                ${${message_interface}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS} )


add_executable(hik_camera_app src/hik_camera_app.cpp)
target_link_libraries(hik_camera_app hik_tem)

catkin_install_python(PROGRAMS
  scripts/ptz_ctrl.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
