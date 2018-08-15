cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kitti2pcd)

find_package(PCL 1.8 REQUIRED)

add_compile_options(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kitti2pcd kitti2pcd.cpp)
target_link_libraries (kitti2pcd ${PCL_LIBRARIES})
