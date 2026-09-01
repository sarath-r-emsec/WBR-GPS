cmake_minimum_required(VERSION 3.16)
project(WBR-GPS C CXX)

# C++17, not 20: WBR-SA is C++17 and includes our client headers.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

option(WBR_GPS_COVERAGE "Build with gcov instrumentation" OFF)
if(WBR_GPS_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Shared between the daemon and the client library.
add_library(wbr_gps_core STATIC
  src/gps_types.cpp
  src/nmea_parser.cpp
  src/json_io.cpp
  src/state_store.cpp
  src/serial_source.cpp
  src/device_presence.cpp
  src/hid_source.cpp
)

enable_testing()

add_executable(test_types tests/test_types.cpp)
target_include_directories(test_types PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_types PRIVATE wbr_gps_core)
add_test(NAME types COMMAND test_types)

add_executable(test_nmea_parser tests/test_nmea_parser.cpp)
target_include_directories(test_nmea_parser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_nmea_parser PRIVATE wbr_gps_core)
add_test(NAME nmea_parser COMMAND test_nmea_parser)

add_executable(test_json_io tests/test_json_io.cpp)
target_include_directories(test_json_io PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_json_io PRIVATE wbr_gps_core)
add_test(NAME json_io COMMAND test_json_io)

add_executable(test_state_store tests/test_state_store.cpp)
target_include_directories(test_state_store PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_state_store PRIVATE wbr_gps_core)
add_test(NAME state_store COMMAND test_state_store)

add_executable(test_serial_source tests/test_serial_source.cpp)
target_include_directories(test_serial_source PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_serial_source PRIVATE wbr_gps_core util)
add_test(NAME serial_source COMMAND test_serial_source)

add_executable(test_device_presence tests/test_device_presence.cpp)
target_include_directories(test_device_presence PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_device_presence PRIVATE wbr_gps_core)
add_test(NAME device_presence COMMAND test_device_presence)
