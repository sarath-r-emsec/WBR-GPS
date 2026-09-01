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
