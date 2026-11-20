cmake_minimum_required(VERSION 3.16)
project(tianyi_loco_sdk LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT IS_DIRECTORY "${SLAMWARE_SDK_ROOT}/include/rpos")
  message(FATAL_ERROR "SLAMWARE_SDK_ROOT must point to the official SDK")
endif()
add_executable(tianyi-loco-sdk loco_sdk.cpp)
target_include_directories(tianyi-loco-sdk SYSTEM PRIVATE "${SLAMWARE_SDK_ROOT}/include")
set(sdk_libraries rpos_robotplatforms_rpslamware rpos_framework base64 rlelib
    jsoncpp curl cares ssl crypto boost_atomic boost_chrono boost_date_time
    boost_regex boost_filesystem boost_system boost_thread boost_random z)
set(sdk_archives)
foreach(lib IN LISTS sdk_libraries)
  list(APPEND sdk_archives "${SLAMWARE_SDK_ROOT}/lib/lib${lib}.a")
endforeach()
target_link_libraries(tianyi-loco-sdk PRIVATE "-Wl,--start-group" ${sdk_archives}
    "-Wl,--end-group" pthread dl rt)
install(TARGETS tianyi-loco-sdk RUNTIME DESTINATION bin)
