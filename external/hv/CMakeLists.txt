cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -w -Ofast" CACHE STRING "" FORCE)

#Compiling external libraries

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(hypervol SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/hypervol.c")
