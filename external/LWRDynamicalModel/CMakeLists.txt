cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -w -Ofast" CACHE STRING "" FORCE)

project(LWRDYNAMICALMODEL)

set(CMAKE_CXX_COMPILER /usr/bin/c++)

#Directories of include

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/")

add_library(LWR_DYNAMICAL_MODEL SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/LWR_Dynamic_Model_Lib.cpp" )

