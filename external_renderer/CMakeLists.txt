cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(gz_external_renderer LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Allow running cmake directly against rotary's install/ and .pixi/envs/default/
list(APPEND CMAKE_PREFIX_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../install"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../.pixi/envs/default"
)

find_package(gz-sim REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz-common REQUIRED COMPONENTS graphics)
find_package(gz-math REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(sdformat REQUIRED)
find_package(zenohc REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Qml Widgets)

set(COMMON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")

# ---------------------------------------------------------------------------
# 1. gz-sim System Plugin: gz-sim-external-renderer-system
# ---------------------------------------------------------------------------
add_library(gz-sim-external-renderer-system SHARED
  gz_sim_plugin/ExternalRendererSystem.hh
  gz_sim_plugin/ExternalRendererSystem.cc
)
target_include_directories(gz-sim-external-renderer-system PRIVATE
  "${COMMON_INCLUDE_DIR}"
)
target_link_libraries(gz-sim-external-renderer-system PRIVATE
  gz-sim::gz-sim
  gz-common::gz-common
  gz-common::graphics
  gz-math::gz-math
  gz-plugin::register
  sdformat::sdformat
  zenohc::lib
)

# ---------------------------------------------------------------------------
# 2. gz-gui Plugin: ExternalRendererGui
#    (Library name MUST match class name & QRC prefix per gz-gui convention)
# ---------------------------------------------------------------------------
set(GUI_PLUGIN_SOURCES
  gz_gui_plugin/ExternalRendererGui.hh
  gz_gui_plugin/ExternalRendererGui.cc
  gz_gui_plugin/MetalTextureBridge.hh
  gz_gui_plugin/ExternalRendererGui.qrc
)

set(GUI_PLATFORM_LIBS "")
if (APPLE)
  list(APPEND GUI_PLUGIN_SOURCES
    common/gpu_surface_ipc_macos.mm
    gz_gui_plugin/MetalTextureBridge.mm
  )
  set_source_files_properties(
    common/gpu_surface_ipc_macos.mm
    gz_gui_plugin/MetalTextureBridge.mm
    PROPERTIES
    COMPILE_FLAGS "-fobjc-arc -Wno-deprecated-declarations -Wno-nullability-completeness"
  )
  list(APPEND GUI_PLATFORM_LIBS
    "-framework Foundation"
    "-framework CoreFoundation"
    "-framework CoreVideo"
    "-framework IOSurface"
    "-framework Metal"
  )
endif()

add_library(ExternalRendererGui SHARED ${GUI_PLUGIN_SOURCES})
target_include_directories(ExternalRendererGui PRIVATE
  "${COMMON_INCLUDE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/gz_gui_plugin"
)
target_link_libraries(ExternalRendererGui PRIVATE
  gz-gui::gz-gui
  gz-common::gz-common
  gz-math::gz-math
  gz-plugin::register
  zenohc::lib
  Qt6::Core
  Qt6::Gui
  Qt6::Quick
  Qt6::Qml
  Qt6::Widgets
  ${GUI_PLATFORM_LIBS}
)

# ---------------------------------------------------------------------------
# 3. Native Zenoh SHM + Mach IOSurface Bridge for Blender 5.0 (ctypes .dylib)
# ---------------------------------------------------------------------------
if (APPLE)
  add_library(gz_blender_bridge SHARED
    blender_bridge/gz_blender_bridge.mm
  )
  set_source_files_properties(
    blender_bridge/gz_blender_bridge.mm
    PROPERTIES
    COMPILE_FLAGS "-fobjc-arc -Wno-deprecated-declarations -Wno-nullability-completeness"
  )
  target_include_directories(gz_blender_bridge PRIVATE "${COMMON_INCLUDE_DIR}")
  target_link_libraries(gz_blender_bridge PRIVATE
    zenohc::lib
    "-framework Foundation"
    "-framework CoreFoundation"
    "-framework CoreGraphics"
    "-framework CoreVideo"
    "-framework ImageIO"
    "-framework UniformTypeIdentifiers"
    "-framework IOSurface"
    "-framework Metal"
  )
endif()

# ---------------------------------------------------------------------------
# 4. End-to-End Cross-Process GPU Swapchain + Zenoh SHM Consumer Verification Test
# ---------------------------------------------------------------------------
if (APPLE)
  add_executable(test_gpu_ipc_consumer
    tests/test_gpu_ipc_consumer.mm
    common/gpu_surface_ipc_macos.mm
  )
  set_source_files_properties(
    tests/test_gpu_ipc_consumer.mm
    common/gpu_surface_ipc_macos.mm
    PROPERTIES
    COMPILE_FLAGS "-fobjc-arc -Wno-deprecated-declarations -Wno-nullability-completeness"
  )
  target_include_directories(test_gpu_ipc_consumer PRIVATE "${COMMON_INCLUDE_DIR}")
  target_link_libraries(test_gpu_ipc_consumer PRIVATE
    zenohc::lib
    ${GUI_PLATFORM_LIBS}
  )
endif()
