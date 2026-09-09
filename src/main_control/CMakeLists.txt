cmake_minimum_required(VERSION 3.10)
project(main_control)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  raicom_vision_laser
  mavros_msgs
  nav_msgs
  tf
)

catkin_package(
    CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    raicom_vision_laser
    mavros_msgs
    nav_msgs
)

# 包含头文件目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/../devel/include  # for pcl_detection2/SquareRing.h
)

# 单文件过程式主控
add_executable(main_control src/main_control.cpp)
target_link_libraries(main_control ${catkin_LIBRARIES})
add_dependencies(main_control ${catkin_EXPORTED_TARGETS} pcl_detection2_generate_messages_cpp) 
