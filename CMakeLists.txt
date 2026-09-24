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
include(GNUInstallDirs)

# ON (default): install udev rules, tmpfiles.d config and a system-wide
# systemd unit -- needs root at install time, matches install.sh's current
# behavior, socket lives at the compiled-in /run/wbr-gps (tmpfiles.d-managed).
#
# OFF: no root needed ANYWHERE, not for configure, build or install. Installs
# a per-user systemd unit instead (WantedBy=default.target, `systemctl --user
# enable --now wbr-gpsd`), pinned via Environment=WBR_GPS_SOCKET_DIR=%t/wbr-gps
# (%t = $XDG_RUNTIME_DIR, always present, tmpfs, owned by the invoking user --
# see gps_types.cpp's default_socket_path()/default_lock_path()). The daemon
# creates that directory itself on first bind (server.cpp's
# ensure_parent_dir()), so there is no tmpfiles.d equivalent to install. No
# udev rule is installed in this mode -- see the message() below for exactly
# what that costs and why none of it is fatal.
#
# Set CMAKE_INSTALL_PREFIX (at CONFIGURE time, e.g.
# `cmake -DCMAKE_INSTALL_PREFIX=$HOME/prefix ...`) for a non-system prefix;
# the per-user unit's ExecStart is baked from CMAKE_INSTALL_FULL_BINDIR at
# configure time, so passing --prefix only to `cmake --install` later would
# generate a unit pointing at the wrong binary path.
option(WBR_GPS_SYSTEM_SERVICE
  "Install udev rules, tmpfiles.d config and a SYSTEM systemd unit (needs root). OFF = per-user unit, no root anywhere." ON)

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
install(TARGETS wbr-gpsd RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime)

# ---------------------------------------------------------------------------
# Service integration -- everything install.sh used to do with raw `install`
# calls now lives here, so `cmake --install build` (root) or
# `cmake --install build` (no root, WBR_GPS_SYSTEM_SERVICE=OFF) does the
# whole job; install.sh is now a thin wrapper (pre-flight verify, the
# 99-leobodnar.rules migration, and the post-install smoke test, none of
# which belong in a CMake install() rule).
#
# Two COMPONENTs (runtime = the binary, service = udev/tmpfiles/systemd unit
# + their reload hooks), not because anyone installs them separately in
# normal use -- a plain `cmake --install build` with no --component installs
# both -- but so install.sh can install the binary FIRST, then run
# `systemd-analyze verify` against a unit file whose ExecStart now resolves
# to a real, just-installed binary, before touching udev/tmpfiles/systemd at
# all: `cmake --install build --component runtime` then
# `cmake --install build --component service`.
# ---------------------------------------------------------------------------
if(WBR_GPS_SYSTEM_SERVICE)
  install(FILES udev/99-wbr-gps.rules DESTINATION /etc/udev/rules.d COMPONENT service)
  install(FILES tmpfiles/wbr-gps.conf DESTINATION /etc/tmpfiles.d COMPONENT service)
  install(FILES systemd/wbr-gpsd.service DESTINATION /etc/systemd/system COMPONENT service)

  # Materialize right now instead of waiting for the next boot/udev event --
  # same reasoning install.sh already had for these three calls. Each one
  # checks RESULT_VARIABLE and fails the install loudly on error: CMake
  # 3.16 (this project's minimum) predates COMMAND_ERROR_IS_FATAL (3.19),
  # and execute_process() otherwise swallows a nonzero exit silently, which
  # would regress the old script's `set -euo pipefail` fail-loud behavior.
  install(CODE [[
    execute_process(COMMAND systemd-tmpfiles --create /etc/tmpfiles.d/wbr-gps.conf RESULT_VARIABLE _r)
    if(NOT _r EQUAL 0)
      message(FATAL_ERROR "systemd-tmpfiles --create failed (exit code ${_r})")
    endif()
  ]] COMPONENT service)
  install(CODE [[
    execute_process(COMMAND udevadm control --reload-rules RESULT_VARIABLE _r)
    if(NOT _r EQUAL 0)
      message(FATAL_ERROR "udevadm control --reload-rules failed (exit code ${_r})")
    endif()
  ]] COMPONENT service)
  install(CODE [[
    execute_process(COMMAND udevadm trigger --subsystem-match=tty --subsystem-match=hidraw RESULT_VARIABLE _r)
    if(NOT _r EQUAL 0)
      message(FATAL_ERROR "udevadm trigger failed (exit code ${_r})")
    endif()
  ]] COMPONENT service)
  install(CODE [[
    execute_process(COMMAND systemctl daemon-reload RESULT_VARIABLE _r)
    if(NOT _r EQUAL 0)
      message(FATAL_ERROR "systemctl daemon-reload failed (exit code ${_r})")
    endif()
  ]] COMPONENT service)
else()
  message(STATUS
    "WBR_GPS_SYSTEM_SERVICE=OFF: installing a PER-USER systemd unit, no root "
    "needed anywhere (configure, build, or install). Trade-offs vs the "
    "system install, all non-fatal -- the daemon degrades, it does not fail "
    "to start:\n"
    "  (1) no udev rule -> ModemManager may still probe the GPSDO's tty\n"
    "  (2) no /dev/gpsdo symlink -> relies on device-presence scanning only\n"
    "  (3) hidraw (GPSDO lock status) is likely root-only without a rule "
    "granting your group access, so hid_ok/gpsdo_locked will probably stay "
    "false; NMEA/serial is unaffected.\n"
    "Socket/lock file live under $XDG_RUNTIME_DIR/wbr-gps, created by the "
    "daemon itself on first bind -- nothing to pre-create.")

  configure_file(systemd/wbr-gpsd.user.service.in
                 ${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd.service @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd.service
          DESTINATION $ENV{HOME}/.config/systemd/user COMPONENT service)
  install(CODE [[
    execute_process(COMMAND systemctl --user daemon-reload RESULT_VARIABLE _r)
    if(NOT _r EQUAL 0)
      message(FATAL_ERROR "systemctl --user daemon-reload failed (exit code ${_r})")
    endif()
  ]] COMPONENT service)
endif()

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

# tools/gpsq is a debug tool, but three test/deploy scripts call it
# positionally, so its argument parsing is load-bearing.
add_test(NAME gpsq_args
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_gpsq_args.py)
