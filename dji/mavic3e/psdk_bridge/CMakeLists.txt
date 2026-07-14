cmake_minimum_required(VERSION 3.10)
project(psdk_bridge C)

set(CMAKE_C_STANDARD 11)

# ── PSDK library path (set via -DPSDK_DIR=... or env) ─────────────────────
set(PSDK_DIR "$ENV{PSDK_DIR}" CACHE PATH "Path to Payload-SDK root")
if(NOT PSDK_DIR)
    set(PSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Payload-SDK")
endif()

# ── PSDK headers & lib ────────────────────────────────────────────────────
include_directories(
    ${PSDK_DIR}/psdk_lib/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# PSDK pre-built static library
# Search paths: platform-specific lib dirs in PSDK distribution
find_library(PSDK_LIB payloadsdk
    PATHS
        ${PSDK_DIR}/psdk_lib/lib/aarch64-linux-gnu-gcc
        ${PSDK_DIR}/psdk_lib/lib/x86_64-linux-gnu-gcc
        ${PSDK_DIR}/psdk_lib/lib/arm-linux-gnueabihf-gcc
        ${PSDK_DIR}/build/psdk_lib
    NO_DEFAULT_PATH
)
if(NOT PSDK_LIB)
    message(WARNING "libpayloadsdk.a not found in ${PSDK_DIR}/psdk_lib/lib/ — using stub")
    set(PSDK_LIB "")
endif()

# ── System dependencies ──────────────────────────────────────────────────
find_package(Threads REQUIRED)

# libusb for E-Port USB communication
find_library(LIBUSB_LIB usb-1.0)
find_path(LIBUSB_INCLUDE libusb-1.0/libusb.h)
if(LIBUSB_INCLUDE)
    include_directories(${LIBUSB_INCLUDE})
endif()

# OpenSSL for PSDK auth (optional in stub mode)
find_package(OpenSSL QUIET)
if(OPENSSL_FOUND)
    include_directories(${OPENSSL_INCLUDE_DIR})
endif()

# ── Sources ──────────────────────────────────────────────────────────────
set(SOURCES
    main.c
    ipc.c
    hal_uart.c
    hal_network.c
    hal_usb_bulk.c
    osal_posix.c
    osal_socket.c
    telemetry.c
    flight_ctrl.c
    camera_mgr.c
    gimbal_mgr.c
    liveview.c
    waypoint.c
    perception.c
    speaker.c
    hms.c
    time_sync.c
)

add_executable(psdk_bridge ${SOURCES})

target_link_libraries(psdk_bridge
    ${PSDK_LIB}
    Threads::Threads
    ${LIBUSB_LIB}
    avcodec avformat avutil swscale
    jpeg
    m
)

if(OPENSSL_FOUND)
    target_link_libraries(psdk_bridge ${OPENSSL_LIBRARIES})
endif()

# startup_bulk — initializes FunctionFS endpoints
add_executable(startup_bulk startup_bulk.c)
target_link_libraries(startup_bulk)

# ── Install ──────────────────────────────────────────────────────────────
install(TARGETS psdk_bridge startup_bulk DESTINATION bin)
