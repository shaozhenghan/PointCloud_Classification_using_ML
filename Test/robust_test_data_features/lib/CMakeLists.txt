cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_compile_options(-std=c++11)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_VERBOSE_MAKEFILE on)

#message(${PROJECT_SOURCE_DIR})

set(LIB_SRC accessFile.cpp extractFeature.cpp sampleCloud.cpp searchKdtree.cpp getFeatureVector.cpp gen_robust_test_data.cpp get_file_name.c)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library(libfeature SHARED ${LIB_SRC}) #dll
add_library(libfeature ${LIB_SRC}) #lib

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set_target_properties(libfeature PROPERTIES OUTPUT_NAME "feature")
