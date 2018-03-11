cmake_minimum_required(VERSION 2.8)

project (demofct)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(EIGEN3_INCLUDE_DIR  "/usr/include/eigen3" )
#set(CMAKE_INSTALL_LIBDIR "../lib")
set(CMAKE_INSTALL_PREFIX "../..")

add_definitions(-Wextra -Wall)

include_directories(../include ${EIGEN3_INCLUDE_DIR})
add_library(demofct SHARED demofct.cpp)

install(TARGETS demofct DESTINATION "lib")
#PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
