cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -w -g -Ofast" CACHE STRING "" FORCE)

project(LWRDYNAMICALMODEL)

set(CMAKE_CXX_COMPILER /usr/bin/c++)
set (PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set (PROJECT_BINARY_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set (EXECUTABLE_OUTPUT_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/bin")


#Directories of include

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/")

add_library(tf_network SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/tf_network.cpp" )

target_link_libraries(tf_network PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/libtensorflow_framework.so" "${CMAKE_CURRENT_SOURCE_DIR}/lib/libtensorflow.so")

add_executable(main "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

target_link_libraries(main tf_network)

