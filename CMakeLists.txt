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

# Tests (no sensor needed): ctest --test-dir build
enable_testing()

add_executable(test_imu_timestamper tests/test_imu_timestamper.cpp)
set_target_properties(test_imu_timestamper PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_compile_options(test_imu_timestamper PRIVATE -O2)
add_test(NAME imu_timestamper COMMAND test_imu_timestamper)

add_executable(test_datawriter tests/test_datawriter.cpp)
set_target_properties(test_datawriter PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_datawriter laszip)
add_test(NAME datawriter COMMAND test_datawriter)
