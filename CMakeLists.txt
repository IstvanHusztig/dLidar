cmake_minimum_required(VERSION 3.0)
project(dLidar)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

include_directories(include 3rd/include 3rd)

add_subdirectory(3rd)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

add_executable(dLidar src/dLidar.cpp)
target_link_libraries(dLidar laszip  ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a)