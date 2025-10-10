cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(global_path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_ros
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)  # 添加Eigen3依赖

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_path_planner
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  # 添加Eigen头文件路径
)

## 创建一个只包含头文件的库
add_library(${PROJECT_NAME} INTERFACE)

## 添加库的包含目录
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

## 添加可执行文件
add_executable(rrt_interactive_node src/rrt_interactive_node.cpp)
add_executable(rrt_star_interactive_node src/rrt_star_interactive_node.cpp)

## 链接库
target_link_libraries(rrt_interactive_node 
  ${PROJECT_NAME}  # 链接到头文件库
  ${catkin_LIBRARIES}
)

target_link_libraries(rrt_star_interactive_node 
  ${PROJECT_NAME}  # 链接到头文件库
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Install the export set for use by downstream projects
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)