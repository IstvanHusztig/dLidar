cmake_minimum_required(VERSION 3.0)
project(dLidar)

set(CMAKE_BUILD_TYPE "Debug")
# No fused multiply-add: results must not depend on -O level or how code is
# split (bin/dLidar is -O0, the replay tools -O2; tools/golden_replay.sh).
set(CMAKE_CXX_FLAGS "-std=c++17 -ffp-contract=off")
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

add_executable(test_time_alignment tests/test_time_alignment.cpp)
set_target_properties(test_time_alignment PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_compile_options(test_time_alignment PRIVATE -O2)
add_test(NAME time_alignment COMMAND test_time_alignment)

# dLidar's points vs the vendor SDK's own conversion (loopback UDP, ports 16101/16201).
add_executable(test_sdk_conversion tests/test_sdk_conversion.cpp)
set_target_properties(test_sdk_conversion PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_sdk_conversion ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a pthread)
add_test(NAME sdk_conversion COMMAND test_sdk_conversion)

# Replays a recording's packetsNNNN.bin through the IMU timestamping.
add_executable(replay_packets tools/replay_packets.cpp)
target_compile_options(replay_packets PRIVATE -O2)

# Replays a recording's packetsNNNN.bin with synthetic packet contents into a
# new recording (tools/compare_recording_formats.sh compares formats with it).
add_executable(replay_capture tools/replay_capture.cpp)
target_compile_options(replay_capture PRIVATE -O2)
target_link_libraries(replay_capture laszip)

# Layer dependency rule (docs/architecture.md); --self-test checks the checker.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
add_test(NAME architecture COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/check_architecture.py ${PROJECT_SOURCE_DIR}/src)
add_test(NAME architecture_selftest COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/check_architecture.py --self-test)
endif()
