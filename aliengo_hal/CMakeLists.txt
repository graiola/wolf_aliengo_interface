# References:
# https://stackoverflow.com/questions/56135785/correctly-set-the-location-of-imported-cmake-targets-for-an-installed-package

cmake_minimum_required(VERSION 3.5)
project(aliengo_hal)

# Ensures CMake creates the file compile_commands.json (Required for Vim-YCM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(VERSION 0.1.0)

option(BUILD_ALIENGOHAL_TEST "Build the tests" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INCLUDE_INSTALL_DIR include)  # include
# set(LIBRARY_INSTALL_DIR lib/aliengohal)
set(LIBRARY_INSTALL_DIR lib)
set(ConfigFileInstallDir lib/cmake/aliengohal)


# ###################
# ALIENGO HAL LIBRARY
# ###################

# Unitree-SDK dependency
find_package(unitreesdk QUIET)
#if(NOT unitreesdk_FOUND)
  # Compile and install unitreesdk if it is not already installed
  add_subdirectory(third_party/unitree_legged_sdk)
#endif()

# add_library(aliengohal STATIC  # TODO: decide if want it static or dynamic
add_library(aliengohal SHARED
  src/aliengo_hal.cpp
)

target_include_directories(aliengohal
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>   # <prefix>/include
    # $<INSTALL_INTERFACE:$<$<CONFIG:Debug>:$<INSTALL_PREFIX>/include>>
)

target_link_libraries(aliengohal
  # PRIVATE
  PUBLIC
    unitreesdk
)


set_property(TARGET aliengohal PROPERTY CXX_STANDARD 11)  # At least std c++11

# # TODO: this is not necesary when using a shared library. Uncomment it if we decide to make the library static
# set_property(TARGET aliengohal PROPERTY POSITION_INDEPENDENT_CODE ON)


# ##############
# PY ALIENGO HAL
# ##############

# Pybind11 dependency
add_subdirectory(third_party/pybind11)

pybind11_add_module(pyaliengo_hal
  src/python_interface.cpp
)
target_link_libraries(pyaliengo_hal
  PUBLIC
    aliengohal
    pybind11::embed
)


# #####
# TESTS
# #####
if(BUILD_ALIENGOHAL_TEST)
  add_subdirectory(test)
endif()
 
 
# #######
# INSTALL
# #######
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS
  aliengohal
  EXPORT aliengohalTargets
  RUNTIME DESTINATION bin COMPONENT Runtime
  LIBRARY DESTINATION lib COMPONENT Runtime
  ARCHIVE DESTINATION lib COMPONENT Development
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE DESTINATION bin COMPONENT Runtime  
)
# Get python-site directory. Ref: https://stackoverflow.com/questions/1242904/finding-python-site-packages-directory-with-cmake
if(NOT PYTHON_SITE)
  # execute_process(
  #   COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
  #     from distutils import sysconfig as sc
  #     print(sc.get_python_lib(prefix='', plat_specific=True))"
  #   OUTPUT_VARIABLE PYTHON_SITE
  #   OUTPUT_STRIP_TRAILING_WHITESPACE
  # )
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
      from distutils import sysconfig as sc
      print(sc.get_python_lib(prefix='', standard_lib=True, plat_specific=True))"
      # print(sc.get_python_lib(prefix='', standard_lib=True, plat_specific=True)+'/dist-packages')"
    OUTPUT_VARIABLE PYTHON_SITE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
install(TARGETS
  pyaliengo_hal
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_SITE}"
)
unset(PYTHON_SITE CACHE)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/aliengohalConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/aliengohalConfig.cmake"
  INSTALL_DESTINATION "${ConfigFileInstallDir}"
  PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR
)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/aliengohalConfigVersion.cmake"
  VERSION ${VERSION}
  COMPATIBILITY AnyNewerVersion
  # COMPATIBILITY ExactVersion
)

export( # Creates Targets.cmake for build tree
  EXPORT aliengohalTargets
  FILE "${PROJECT_BINARY_DIR}/aliengohalTargets.cmake"
)
install( # Creates Targets.cmake referencing the relocatable install location
  EXPORT aliengohalTargets
  FILE aliengohalTargets.cmake
  # DESTINATION lib/cmake  # TODO: Is it ok this? or do we need
  DESTINATION "${ConfigFileInstallDir}"
)
install(FILES
  "${PROJECT_BINARY_DIR}/aliengohalConfig.cmake"
  "${PROJECT_BINARY_DIR}/aliengohalConfigVersion.cmake"
  # "${PROJECT_BINARY_DIR}/unitreesdkTargets.cmake"
  DESTINATION "${ConfigFileInstallDir}"
)

