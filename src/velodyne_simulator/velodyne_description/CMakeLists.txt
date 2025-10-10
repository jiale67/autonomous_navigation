cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(velodyne_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes rviz urdf world
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
