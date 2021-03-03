cmake_minimum_required(VERSION 3.5)

project(rviz2_views_move_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99) 
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()



# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)  # provided by tinyxml_vendor
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 QUIET)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz2_views_move_plugin_headers_to_moc
  include/orbit_view_controller.hpp
)

set(rviz2_views_move_plugin_source_files
  src/orbit_view_controller.cpp
)

add_library(rviz2_views_move_plugin SHARED
  ${rviz2_views_move_plugin_headers_to_moc}
  ${rviz2_views_move_plugin_source_files}
)

target_include_directories(rviz2_views_move_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rviz_rendering_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${map_msgs_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${resource_retriever_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

target_link_libraries(rviz2_views_move_plugin
  laser_geometry::laser_geometry
  resource_retriever::resource_retriever
  rviz_common::rviz_common
  -lpthread
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz2_views_move_plugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(rviz2_views_move_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz2_views_move_plugin
  geometry_msgs
  laser_geometry
  nav_msgs
  map_msgs
  rclcpp
  resource_retriever
  urdf
  visualization_msgs
)

ament_export_include_directories(include)
ament_export_interfaces(rviz2_views_move_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  urdf
  visualization_msgs
)

install(
  TARGETS rviz2_views_move_plugin
  EXPORT rviz2_views_move_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # TODO(wjwwood): replace this with ament_lint_auto() and/or add the copyright linter back
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  set(TEST_INCLUDE_DIRS
    ${map_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
  )
  ament_include_directories_order(TEST_INCLUDE_DIRS ${TEST_INCLUDE_DIRS})

  set(TEST_LINK_LIBRARIES
    rviz2_views_move_plugin
    Qt5::Widgets
    ${rclcpp_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${map_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
  )

endif()

ament_package()
