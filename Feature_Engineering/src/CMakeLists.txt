cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_compile_options(-std=c++11)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_VERBOSE_MAKEFILE on)

#message(${PROJECT_SOURCE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/lib)

set(APP_SRC1 testModule.cpp)
set(APP_SRC2 analyseFeature.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(testModule ${APP_SRC1})
add_executable(analyseFeature ${APP_SRC2})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

target_link_libraries(testModule libfeature ${PCL_LIBRARIES})
target_link_libraries(analyseFeature libfeature ${PCL_LIBRARIES})
