cmake_minimum_required(VERSION 3.0)
project(dLidar)

set(CMAKE_BUILD_TYPE "Debug")
# No fused multiply-add: results must not depend on -O level or how code is
# split (bin/dLidar is -O0, the replay tools -O2; tools/golden_replay.sh).
set(CMAKE_CXX_FLAGS "-std=c++17 -ffp-contract=off")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

add_subdirectory(3rd)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

set(UNITREE_SDK_LIB ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a)

# Layers (docs/architecture.md). dlidar_core = src/domain + src/application:
# include path src/ only, no vendor SDK, no LASzip, nothing to link. Which
# headers each layer may include is checked by ctest architecture.
add_library(dlidar_core INTERFACE)
target_include_directories(dlidar_core INTERFACE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(dlidar_core INTERFACE pthread)

# Unitree L2 adapter (src/adapters/unitree_l2): the only code that sees the vendor SDK.
add_library(dlidar_l2 STATIC
    src/adapters/unitree_l2/l2_conversion.cpp
    src/adapters/unitree_l2/live_l2_source.cpp
    src/adapters/unitree_l2/synthetic_l2_source.cpp
    src/adapters/unitree_l2/unitree_l2_sensor.cpp)
set_target_properties(dlidar_l2 PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_link_libraries(dlidar_l2 PUBLIC dlidar_core ${UNITREE_SDK_LIB})

# Livox Mid-360S adapter (src/adapters/livox_mid360s): the only code that sees
# Livox-SDK2 (git submodule external/Livox-SDK2).
set(LIVOX_SDK_DIR ${PROJECT_SOURCE_DIR}/external/Livox-SDK2)
# Only the SDK's static library (sdk_core), not its samples or shared library;
# its own warnings are not ours.
add_subdirectory(${LIVOX_SDK_DIR}/sdk_core ${PROJECT_BINARY_DIR}/livox_sdk EXCLUDE_FROM_ALL)
set_target_properties(livox_lidar_sdk_static PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_compile_options(livox_lidar_sdk_static PRIVATE -w)
add_library(dlidar_livox STATIC
    src/adapters/livox_mid360s/livox_conversion.cpp
    src/adapters/livox_mid360s/livox_mid360s_sensor.cpp
    src/adapters/livox_mid360s/live_livox_source.cpp
    src/adapters/livox_mid360s/synthetic_livox_source.cpp)
set_target_properties(dlidar_livox PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_include_directories(dlidar_livox PUBLIC ${LIVOX_SDK_DIR}/include)
target_link_libraries(dlidar_livox PUBLIC dlidar_core PRIVATE livox_lidar_sdk_static)

# Bluetooth adapter (src/adapters/bluetooth): the pairing window over BlueZ
# D-Bus. GIO/GLib only here and in dlidar_phone.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GIO REQUIRED IMPORTED_TARGET gio-2.0 gio-unix-2.0)
add_library(dlidar_bluetooth STATIC src/adapters/bluetooth/pairing_window.cpp)
set_target_properties(dlidar_bluetooth PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_link_libraries(dlidar_bluetooth PUBLIC dlidar_core PRIVATE PkgConfig::GIO)

# nlohmann/json (submodule external/json, v3.11.3), headers only.
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json SYSTEM INTERFACE ${PROJECT_SOURCE_DIR}/external/json/single_include)

# Phone link adapter (src/adapters/phone, openspec phone-link-core): the
# protocol, its GLib loop and transports. Only it sees nlohmann/json.
add_library(dlidar_phone STATIC src/adapters/phone/protocol_codec.cpp src/adapters/phone/phone_server.cpp
    src/adapters/phone/phone_loop.cpp src/adapters/phone/socket_transport.cpp src/adapters/phone/phone_link.cpp
    src/adapters/phone/ble_link.cpp)
set_target_properties(dlidar_phone PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_link_libraries(dlidar_phone PUBLIC dlidar_core PRIVATE nlohmann_json PkgConfig::GIO)

# build/generated/dlidar_version.h (git describe), refreshed on every build;
# only src/app/main.cpp includes it.
add_custom_target(dlidar_version
    COMMAND ${CMAKE_COMMAND} -DSRC=${PROJECT_SOURCE_DIR} -DOUT=${PROJECT_BINARY_DIR}/generated/dlidar_version.h
            -P ${PROJECT_SOURCE_DIR}/cmake/dlidar_version.cmake
    BYPRODUCTS ${PROJECT_BINARY_DIR}/generated/dlidar_version.h)

# GPIO adapter (src/adapters/gpio): push button and status LED on the Pi's
# header; the only code that sees <linux/gpio.h>.
add_library(dlidar_gpio STATIC src/adapters/gpio/gpio_line.cpp src/adapters/gpio/button_input.cpp
    src/adapters/gpio/status_led.cpp)
set_target_properties(dlidar_gpio PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_link_libraries(dlidar_gpio PUBLIC dlidar_core)

# Storage adapter (src/adapters/storage): recording files; the only code that sees LASzip.
add_library(dlidar_storage STATIC
    src/adapters/storage/laz_encoder.cpp
    src/adapters/storage/recovery.cpp)
set_target_properties(dlidar_storage PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_include_directories(dlidar_storage PUBLIC ${PROJECT_SOURCE_DIR}/3rd/include ${PROJECT_SOURCE_DIR}/3rd)
target_link_libraries(dlidar_storage PUBLIC dlidar_core laszip)

# Console output channel (src/adapters/console).
add_library(dlidar_console STATIC src/adapters/console/console_events.cpp)
set_target_properties(dlidar_console PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_link_libraries(dlidar_console PUBLIC dlidar_core)

# The device application: composition root in src/app. dlidar_app holds its
# parts besides main() (settings file), so tests can link them.
add_library(dlidar_app STATIC src/app/settings.cpp src/app/power.cpp)
set_target_properties(dlidar_app PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
target_link_libraries(dlidar_app PUBLIC dlidar_core)

add_executable(dLidar src/app/main.cpp src/app/sensor_factory.cpp)
target_link_libraries(dLidar dlidar_app dlidar_l2 dlidar_livox dlidar_gpio dlidar_bluetooth dlidar_phone dlidar_storage dlidar_console)
add_dependencies(dLidar dlidar_version)
target_include_directories(dLidar PRIVATE ${PROJECT_BINARY_DIR}/generated)

# Tests (no sensor needed): ctest --test-dir build
enable_testing()

add_executable(test_imu_timestamper tests/test_imu_timestamper.cpp)
set_target_properties(test_imu_timestamper PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_compile_options(test_imu_timestamper PRIVATE -O2)
target_link_libraries(test_imu_timestamper dlidar_core)
add_test(NAME imu_timestamper COMMAND test_imu_timestamper)

add_executable(test_datawriter tests/adapters/storage/test_datawriter.cpp)
set_target_properties(test_datawriter PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_datawriter dlidar_l2 dlidar_storage)
add_test(NAME datawriter COMMAND test_datawriter)

add_executable(test_time_alignment tests/adapters/unitree_l2/test_l2_time_alignment.cpp)
set_target_properties(test_time_alignment PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_compile_options(test_time_alignment PRIVATE -O2)
target_link_libraries(test_time_alignment dlidar_l2)
add_test(NAME time_alignment COMMAND test_time_alignment)

# dLidar's points vs the vendor SDK's own conversion (loopback UDP, ports 16101/16201).
add_executable(test_sdk_conversion tests/adapters/unitree_l2/test_sdk_conversion.cpp)
set_target_properties(test_sdk_conversion PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_sdk_conversion dlidar_l2)
add_test(NAME sdk_conversion COMMAND test_sdk_conversion)

# Replays a recording's packetsNNNN.bin through the IMU timestamping.
add_executable(replay_packets tools/replay_packets.cpp)
target_compile_options(replay_packets PRIVATE -O2)
target_link_libraries(replay_packets dlidar_core)

# Replays a recording's packetsNNNN.bin with synthetic packet contents into a
# new recording (tools/compare_recording_formats.sh compares formats with it).
add_executable(replay_capture tools/replay_capture.cpp)
target_compile_options(replay_capture PRIVATE -O2)
target_link_libraries(replay_capture dlidar_l2 dlidar_livox dlidar_storage dlidar_console)

add_executable(test_capture_timing tests/application/test_capture_timing.cpp)
set_target_properties(test_capture_timing PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_capture_timing dlidar_core)
add_test(NAME capture_timing COMMAND test_capture_timing)

foreach(t record_session device_control device_api)
add_executable(test_${t} tests/application/test_${t}.cpp)
set_target_properties(test_${t} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_${t} dlidar_core)
add_test(NAME ${t} COMMAND test_${t})
endforeach()

add_executable(test_l2_conversion tests/adapters/unitree_l2/test_l2_conversion.cpp)
set_target_properties(test_l2_conversion PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_l2_conversion dlidar_l2)
add_test(NAME l2_conversion COMMAND test_l2_conversion)

add_executable(test_button_commands tests/app/test_button_commands.cpp)
set_target_properties(test_button_commands PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_button_commands dlidar_app)
add_test(NAME button_commands COMMAND test_button_commands)

add_executable(test_settings tests/app/test_settings.cpp)
set_target_properties(test_settings PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_settings dlidar_app)
add_test(NAME settings COMMAND test_settings)

foreach(t livox_conversion livox_sensor livox_time_alignment livox_packet_pool)
add_executable(test_${t} tests/adapters/livox_mid360s/test_${t}.cpp)
set_target_properties(test_${t} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_${t} dlidar_livox)
add_test(NAME ${t} COMMAND test_${t})
endforeach()
target_compile_options(test_livox_time_alignment PRIVATE -O2)

foreach(t press_detector button_input status_led)
add_executable(test_${t} tests/adapters/gpio/test_${t}.cpp)
set_target_properties(test_${t} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_${t} dlidar_gpio)
add_test(NAME ${t} COMMAND test_${t})
endforeach()

foreach(t line_framer protocol_codec phone_server phone_socket ble_link)
add_executable(test_${t} tests/adapters/phone/test_${t}.cpp)
set_target_properties(test_${t} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_${t} dlidar_phone nlohmann_json)
add_test(NAME ${t} COMMAND test_${t})
endforeach()
target_compile_definitions(test_protocol_codec PRIVATE DLIDAR_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# Domain unit tests (src/domain only, dlidar_core).
foreach(t axis_map point_filter)
add_executable(test_${t} tests/domain/test_${t}.cpp)
set_target_properties(test_${t} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
target_link_libraries(test_${t} dlidar_core)
add_test(NAME ${t} COMMAND test_${t})
endforeach()

# Throwaway BLE spike (openspec/changes/phone-link-core, task 1); not part of
# dLidar: cmake -DDLIDAR_BLE_SPIKE=ON.
option(DLIDAR_BLE_SPIKE "Build the throwaway BLE spike tools/ble_spike" OFF)
if(DLIDAR_BLE_SPIKE)
pkg_check_modules(SPIKE_GIO REQUIRED IMPORTED_TARGET gio-2.0 gio-unix-2.0)
add_executable(ble_spike tools/ble_spike/ble_spike.cpp)
target_link_libraries(ble_spike PkgConfig::SPIKE_GIO)
endif()

# Layer dependency rule (docs/architecture.md); --self-test checks the checker.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
add_test(NAME architecture COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/check_architecture.py --strict ${PROJECT_SOURCE_DIR}/src)
add_test(NAME architecture_selftest COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/check_architecture.py --self-test)
endif()
