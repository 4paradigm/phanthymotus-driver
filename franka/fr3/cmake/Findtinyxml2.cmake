# Ubuntu 22.04 ships tinyxml2.pc, but no tinyxml2 CMake package configuration.
# Adapt its system package to the target expected by libfranka 0.18.1.
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
if(NOT TARGET tinyxml2)
  add_library(tinyxml2 ALIAS PkgConfig::TINYXML2)
endif()
set(tinyxml2_FOUND TRUE)
