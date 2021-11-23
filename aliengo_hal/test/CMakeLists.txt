cmake_minimum_required(VERSION 3.5)
project(aliengo_hal_test)

find_package(aliengohal QUIET)
# find_package(unitreesdk QUIET)

add_executable(test_aliengo_hal
	test_aliengo_hal.cpp
)

target_link_libraries(test_aliengo_hal
	# PRIVATE
	PUBLIC
		aliengohal
		# unitreesdk
)

set_property(
  TARGET test_aliengo_hal
  PROPERTY
    CXX_STANDARD 11
)

# TODO: Implement the test for python
