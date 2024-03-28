cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -Ofast  -lpthread " CACHE STRING "" FORCE)

project(controller)

#Setting of the dir

set(CMAKE_CXX_COMPILER /usr/bin/c++)
set (PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set (PROJECT_BINARY_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set (EXECUTABLE_OUTPUT_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/bin")
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/limbo/*.hpp")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#Find packages

find_package(Eigen3 REQUIRED)
find_package(TBB 4.2 REQUIRED)
find_package(NLopt 2.6.2 REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 REQUIRED COMPONENTS system filesystem thread regex unit_test_framework)


if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()


#Directories of include

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TBB_INCLUDE_DIR})
include_directories(${TECS_INCLUDE_DIR})
include_directories(${CRYPTOPP_INCLUDE_DIR})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/FRILibrary/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/LWRDynamicalModel/include")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/Tensorflow/include")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/ReflexxesTypeII/include")
include_directories(${NLopt_INCLUDE_DIR})

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/ehvi")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/hv")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/FRILibrary")


link_directories("/usr/local/lib/" ${TBB_LIBRARY})

add_library(controller SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/controller/controller.cpp" )

add_library(controller_kuka STATIC "${CMAKE_CURRENT_SOURCE_DIR}/src/controller/controller_kuka.cpp")

add_library(utils SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/data_utils.cpp")

add_library(kuka_utilities SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/controller/kuka_utilities.cpp")

#add_library(learning SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/learning/learning.cpp")

#add_library(tf_network SHARED "${CMAKE_CURRENT_SOURCE_DIR}/external/Tensorflow/src/tf_network.cpp")

target_link_libraries(utils ${TBB_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(controller ${TBB_LIBRARIES} ${Boost_LIBRARIES} libFastResearchInterfaceLibrary libTypeIRML kuka_utilities)

#target_link_libraries(learning ${TBB_LIBRARIES} ${Boost_LIBRARIES})

#target_link_libraries(tf_network PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/external/Tensorflow/lib/libtensorflow_framework.so" "${CMAKE_CURRENT_SOURCE_DIR}/external/Tensorflow/lib/libtensorflow.so")

add_executable(main "${CMAKE_CURRENT_SOURCE_DIR}/src/main_pos.cpp")

add_executable(main_sim "${CMAKE_CURRENT_SOURCE_DIR}/src/main_sim.cpp")

add_executable(main_sim_DAR "${CMAKE_CURRENT_SOURCE_DIR}/src/main_sim_DAR.cpp")

add_executable(main_DAR "${CMAKE_CURRENT_SOURCE_DIR}/src/main_DAR.cpp")

add_executable(main_DAR_full "${CMAKE_CURRENT_SOURCE_DIR}/src/main_DAR_full.cpp")

add_executable(main_DAR_sat "${CMAKE_CURRENT_SOURCE_DIR}/src/main_DAR_sat.cpp")

add_executable(main_DAR_AW "${CMAKE_CURRENT_SOURCE_DIR}/src/main_DAR_AW.cpp")

add_executable(main_DAR_obs "${CMAKE_CURRENT_SOURCE_DIR}/src/main_DAR_observer.cpp")

#add_executable(main_FBL_PD "${CMAKE_CURRENT_SOURCE_DIR}/src/main_FBL_PD.cpp")

#add_executable(main_SMC "${CMAKE_CURRENT_SOURCE_DIR}/src/main_SMC.cpp")

#add_executable(main_PD "${CMAKE_CURRENT_SOURCE_DIR}/src/main_PD.cpp")

#add_executable(main_sim_noise "${CMAKE_CURRENT_SOURCE_DIR}/src/main_sim_noise.cpp")

#add_executable(main_pos_full_observer "${CMAKE_CURRENT_SOURCE_DIR}/src/main_pos_full_observer.cpp")

#add_executable(main_pos_reduced_observer "${CMAKE_CURRENT_SOURCE_DIR}/src/main_pos_reduced_observer.cpp")

add_executable(main_OPENDIAG "${CMAKE_CURRENT_SOURCE_DIR}/src/main_pos_reduced_observer_OPENDIAG.cpp")

#target_link_libraries(controller_kuka PUBLIC controller utils learning tf_network "${CMAKE_CURRENT_SOURCE_DIR}/external/LWRDynamicalModel/build/libLWR_DYNAMICAL_MODEL.so" "${CMAKE_CURRENT_SOURCE_DIR}/external/ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so" "${CMAKE_CURRENT_SOURCE_DIR}/build/libkuka_utilities.so")

target_link_libraries(controller_kuka PUBLIC controller utils "${CMAKE_CURRENT_SOURCE_DIR}/external/LWRDynamicalModel/build/libLWR_DYNAMICAL_MODEL.so" "${CMAKE_CURRENT_SOURCE_DIR}/build/libkuka_utilities.so")
#target_link_libraries(controller_kuka PUBLIC controller utils "${CMAKE_CURRENT_SOURCE_DIR}/external/LWRDynamicalModel/build/libLWR_DYNAMICAL_MODEL.so" "${CMAKE_CURRENT_SOURCE_DIR}/external/ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so" "${CMAKE_CURRENT_SOURCE_DIR}/build/libkuka_utilities.so")

#add_executable(nn_test "${CMAKE_CURRENT_SOURCE_DIR}/src/nn_test.cpp")

#add_executable(gp "${CMAKE_CURRENT_SOURCE_DIR}/src/gp.cpp")

#target_link_libraries(gp ${TBB_LIBRARIES} ${Boost_LIBRARIES} utils)

target_link_libraries(main controller_kuka)

target_link_libraries(main_sim controller_kuka)

target_link_libraries(main_sim_DAR controller_kuka)

target_link_libraries(main_DAR controller_kuka)

target_link_libraries(main_DAR_full controller_kuka)

target_link_libraries(main_DAR_sat controller_kuka)

target_link_libraries(main_DAR_AW controller_kuka)

target_link_libraries(main_DAR_obs controller_kuka)

#target_link_libraries(main_FBL_PD controller_kuka)

#target_link_libraries(main_SMC controller_kuka)

#target_link_libraries(main_PD controller_kuka)

#target_link_libraries(main_sim_noise controller_kuka)

#target_link_libraries(main_pos_full_observer controller_kuka)

#target_link_libraries(main_pos_reduced_observer controller_kuka)

target_link_libraries(main_OPENDIAG controller_kuka)

#target_link_libraries(nn_test controller_kuka)
