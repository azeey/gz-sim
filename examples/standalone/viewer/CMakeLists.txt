cmake_minimum_required(VERSION 3.16)
project(viewer)

find_package(gz-sim REQUIRED COMPONENTS gui rendering)
find_package(gz-gui REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz-common REQUIRED)
find_package(sdformat REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Quick)

set(CMAKE_CXX_STANDARD 17)

add_executable(viewer viewer.cc)

target_link_libraries(viewer
  PRIVATE
    gz-sim::gui
    gz-sim::rendering
    gz-sim::core
    gz-common::gz-common
    gz-rendering::gz-rendering
    sdformat::sdformat
    Qt6::Core
    Qt6::Quick
)
