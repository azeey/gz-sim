cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-gazebo6 VERSION 6.15.0)
set (GZ_DISTRIBUTION "Fortress")

add_executable(test_segfault test.cc)
add_test(NAME my_unit_test COMMAND test_segfault)
enable_testing()
install(TARGETS test_segfault DESTINATION bin)
