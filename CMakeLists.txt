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
  src/server.cpp
  src/client.cpp
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

add_executable(test_hid_source tests/test_hid_source.cpp)
target_include_directories(test_hid_source PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_hid_source PRIVATE wbr_gps_core util)
add_test(NAME hid_source COMMAND test_hid_source)

add_executable(test_server_protocol tests/test_server_protocol.cpp)
target_include_directories(test_server_protocol PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_server_protocol PRIVATE wbr_gps_core)
add_test(NAME server_protocol COMMAND test_server_protocol)

add_executable(test_server_backpressure tests/test_server_backpressure.cpp)
target_include_directories(test_server_backpressure PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_server_backpressure PRIVATE wbr_gps_core)
add_test(NAME server_backpressure COMMAND test_server_backpressure)

add_executable(test_client_table tests/test_client_table.cpp)
target_include_directories(test_client_table PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/tests
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_client_table PRIVATE wbr_gps_core)
add_test(NAME client_table COMMAND test_client_table)

# The daemon. Everything it needs is in wbr_gps_core; main.cpp is wiring only.
add_executable(wbr-gpsd src/main.cpp)
target_link_libraries(wbr-gpsd PRIVATE wbr_gps_core)
install(TARGETS wbr-gpsd RUNTIME DESTINATION bin)

# Client library test: launches the real wbr-gpsd against a pty, so it
# exercises the actual wire protocol rather than a stub.
add_executable(test_client tests/test_client.cpp)
target_include_directories(test_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_compile_definitions(test_client PRIVATE
  WBR_GPSD_PATH="${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd")
target_link_libraries(test_client PRIVATE wbr_gps_core util pthread)
add_dependencies(test_client wbr-gpsd)
add_test(NAME client COMMAND test_client)

add_executable(test_concurrency tests/test_concurrency.cpp)
target_include_directories(test_concurrency PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_compile_definitions(test_concurrency PRIVATE
  WBR_GPSD_PATH="${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd")
target_link_libraries(test_concurrency PRIVATE wbr_gps_core util pthread)
add_dependencies(test_concurrency wbr-gpsd)
add_test(NAME concurrency COMMAND test_concurrency)

add_executable(test_concurrency_lifecycle tests/test_concurrency_lifecycle.cpp)
target_include_directories(test_concurrency_lifecycle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_compile_definitions(test_concurrency_lifecycle PRIVATE
  WBR_GPSD_PATH="${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd")
target_link_libraries(test_concurrency_lifecycle PRIVATE wbr_gps_core util pthread)
add_dependencies(test_concurrency_lifecycle wbr-gpsd)
add_test(NAME concurrency_lifecycle COMMAND test_concurrency_lifecycle)

# Guarantee G5, checked structurally rather than promised in a comment.
add_test(NAME no_device_access
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_no_device_access.sh)
