# References:
# https://stackoverflow.com/questions/56135785/correctly-set-the-location-of-imported-cmake-targets-for-an-installed-package

cmake_minimum_required(VERSION 3.5)
project(unitree_sdk)

# Ensures CMake creates the file compile_commands.json (Required for YCM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(VERSION 0.1.0)

option(BUILD_UNITREE_SDK_EXAMPLES "Build the Unitree SDK examples" OFF)

set(INCLUDE_INSTALL_DIR include)  # include
# set(LIBRARY_INSTALL_DIR lib/unitreesdk)

set(LIBRARY_INSTALL_DIR lib)
# set(LIBRARY_INSTALL_DIR lib)

set(ConfigFileInstallDir lib/cmake/unitreesdk)

add_library(unitreesdk INTERFACE)

target_include_directories(unitreesdk
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>   # <prefix>/include
    # $<INSTALL_INTERFACE:$<$<CONFIG:Debug>:$<INSTALL_PREFIX>/include>>
)

target_link_libraries(unitreesdk
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64.so>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${LIBRARY_INSTALL_DIR}/libunitree_legged_sdk_amd64.so>
    lcm
)


# ########
# Examples
# ########
if(BUILD_UNITREE_SDK_EXAMPLES)
  add_subdirectory(examples)
endif(BUILD_UNITREE_SDK_EXAMPLES)


# #######
# INSTALL
# #######
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)
install(FILES
  ${PROJECT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64.so
  DESTINATION ${LIBRARY_INSTALL_DIR}
)
install(TARGETS unitreesdk
  EXPORT unitreesdkTargets
  RUNTIME DESTINATION bin COMPONENT Runtime
  LIBRARY DESTINATION lib COMPONENT Runtime
  ARCHIVE DESTINATION lib COMPONENT Development
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE DESTINATION bin COMPONENT Runtime
)

# include(GNUInstallDirs)
# message(STATUS "====>> ${CMAKE_INSTALL_INCLUDEDIR}")
# message(STATUS "====>> ${CMAKE_INSTALL_LIBDIR}")

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/unitreesdkConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/unitreesdkConfig.cmake"
  INSTALL_DESTINATION "${ConfigFileInstallDir}"
  PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR
)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/unitreesdkConfigVersion.cmake"
  VERSION ${VERSION}
  COMPATIBILITY AnyNewerVersion
  # COMPATIBILITY ExactVersion
)

export( # Creates Targets.cmake for build tree
  EXPORT unitreesdkTargets
  FILE "${PROJECT_BINARY_DIR}/unitreesdkTargets.cmake"
)
install( # Creates Targets.cmake referencing the relocatable install location
  EXPORT unitreesdkTargets
  FILE unitreesdkTargets.cmake
  DESTINATION "${ConfigFileInstallDir}"
)
install(FILES
  "${PROJECT_BINARY_DIR}/unitreesdkConfig.cmake"
  "${PROJECT_BINARY_DIR}/unitreesdkConfigVersion.cmake"
  DESTINATION "${ConfigFileInstallDir}"
)
