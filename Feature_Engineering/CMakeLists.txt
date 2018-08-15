cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(feature)

find_package(PCL 1.8 REQUIRED)

add_compile_options(-std=c++11)

#set(CMAKE_VERBOSE_MAKEFILE on)

#message(${PROJECT_SOURCE_DIR})

add_subdirectory(src)

add_subdirectory(lib)
