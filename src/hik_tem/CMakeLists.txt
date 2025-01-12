cmake_minimum_required(VERSION 3.5.0)
project(hik_tem)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  camera_info_manager
  message_generation
  std_msgs
)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDES})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDES})

add_message_files(
  FILES
  PtzCtrl.msg
  P2PData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp std_msgs message_runtime)


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

add_executable(hik_camera_app src/hik_camera_app.cpp)
target_link_libraries(hik_camera_app hik_tem)

catkin_install_python(PROGRAMS
  scripts/ptz_contrl.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
