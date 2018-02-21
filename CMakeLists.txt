# Set the minimum version of cmake required to build this project
cmake_minimum_required(VERSION 2.8)

project(kheops)

find_package(PkgConfig REQUIRED)
set ( EIGEN3_INCLUDE_DIR  "/usr/include/eigen3" )
set(CMAKE_CXX_FLAGS "-std=c++17")

file(GLOB_RECURSE sourcefiles src/*.cpp)
add_definitions(-Wextra -Wall)
include_directories(include  ${EIGEN3_INCLUDE_DIR} )

add_executable(
        kheops
        ${sourcefiles}
)

target_link_libraries(kheops pthread xerces-c dl uuid)
