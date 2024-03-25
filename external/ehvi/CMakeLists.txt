cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -w -Ofast" CACHE STRING "" FORCE)

#Compiling external libraries

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(ehvi_calculations SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/ehvi_calculations.cc")
add_library(ehvi_montecarlo SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/ehvi_montecarlo.cc")
add_library(helper SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/helper.cc")
add_library(ehvi_sliceupdate SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/ehvi_sliceupdate.cc")
add_library(ehvi_hvol SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/ehvi_hvol.cc")
add_library(ehvi_multi SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/ehvi_multi.cc")
