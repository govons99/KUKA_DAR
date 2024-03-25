cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -w -Ofast -D__LINUX__" CACHE STRING "" FORCE)

#Compiling external libraries

file(GLOB LIB "${CMAKE_CURRENT_SOURCE_DIR}/src/FastResearchInterfaceLibrary/*.cpp")

#file(GLOB TEST "${PROJECT_SOURCE_DIR}/src/FastResearchInterfaceTest/*.cpp")

file(GLOB EXAMPLES "${PROJECT_SOURCE_DIR}/src/LWRGettingStartedExamples/*.cpp")
file(GLOB ABSTRACTION "${CMAKE_CURRENT_SOURCE_DIR}/src/LinuxAbstraction/*.cpp")
file(GLOB IRML "${CMAKE_CURRENT_SOURCE_DIR}/src/TypeIRML/*.cpp")

file(GLOB TEST "${CMAKE_CURRENT_SOURCE_DIR}/src/src/FastResearchInterfaceTest/*.cpp")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

#message(${CMAKE_CURRENT_SOURCE_DIR})

#Creation of static libraries to be added in the code
add_library(Linux_abstraction ${ABSTRACTION})
add_library(libFastResearchInterfaceLibrary STATIC ${LIB})
target_link_libraries(libFastResearchInterfaceLibrary Linux_abstraction)
add_library(libTypeIRML STATIC ${IRML})
#add_executable(LWRJointPositionController "${CMAKE_CURRENT_SOURCE_DIR}/src/LWRGettingStartedExamples/LWRJointPositionControlExample.cpp")
#target_link_libraries(LWRJointPositionController libFastResearchInterfaceLibrary libTypeIRML)

#add_executable(LWRJointImpedenceController "${CMAKE_CURRENT_SOURCE_DIR}/src/LWRGettingStartedExamples/LWRJointImpedanceControlExample.cpp")
#target_link_libraries(LWRJointImpedenceController libFastResearchInterfaceLibrary libTypeIRML)

#add_executable(Test "${TEST}")
#target_link_libraries(Test libFastResearchInterfaceLibrary libTypeIRML)
