cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_compile_options(-std=c++11)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_VERBOSE_MAKEFILE on)

#message(${PROJECT_SOURCE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/lib)

set(APP_SRC1 occlusion.cpp)
set(APP_SRC2 sparsity.cpp)
set(APP_SRC3 noise.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(occlusion ${APP_SRC1})
add_executable(sparsity ${APP_SRC2})
add_executable(noise ${APP_SRC3})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

target_link_libraries(occlusion libfeature ${PCL_LIBRARIES})
target_link_libraries(sparsity libfeature ${PCL_LIBRARIES})
target_link_libraries(noise libfeature ${PCL_LIBRARIES})
