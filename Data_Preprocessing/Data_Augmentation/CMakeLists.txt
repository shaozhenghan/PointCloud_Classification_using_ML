cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(data_aug)

find_package(PCL 1.8 REQUIRED)

#add_compile_options(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (data_aug data_augmentation.cpp data_aug.c)
target_link_libraries (data_aug ${PCL_LIBRARIES})
