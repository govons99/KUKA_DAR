cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -Ofast  -lpthread" CACHE STRING "" FORCE)

project(mpc_acado)

#Setting of the dir

set(CMAKE_CXX_COMPILER /usr/bin/c++)
set (PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set (PROJECT_BINARY_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set (EXECUTABLE_OUTPUT_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#Directories of include

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/qpoases/INCLUDE")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/qpoases/SRC")

add_library(acado_integrator SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/acado_integrator.c")

add_library(acado_solver SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/acado_solver.c")

add_library(acado_auxiliary_functions SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/acado_auxiliary_functions.c")

add_library(acado_qpoases_interface SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/acado_qpoases_interface.cpp")