cmake_minimum_required(VERSION 3.5)
project(camera_ros)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

find_package(PkgConfig REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(JPEG REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera)

# library with common utility functions for type conversions
add_library(utils OBJECT
  src/clamp.cpp
  src/compression.cpp
  src/cv_to_pv.cpp
  src/format_mapping.cpp
  src/parameter_conflict_check.cpp
  src/pretty_print.cpp
  src/pv_to_cv.cpp
  src/types.cpp
  src/type_extent.cpp
)
target_include_directories(utils PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(utils JPEG::JPEG)
ament_target_dependencies(
  utils
  "rclcpp"
  "sensor_msgs"
)
set_property(TARGET utils PROPERTY POSITION_INDEPENDENT_CODE ON)

# composable ROS2 node
add_library(camera_component SHARED src/CameraNode.cpp)
rclcpp_components_register_node(camera_component PLUGIN "camera::CameraNode" EXECUTABLE "camera_node")

target_include_directories(camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  camera_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "camera_info_manager"
)

target_include_directories(camera_component PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(camera_component ${libcamera_LINK_LIBRARIES} utils)

install(TARGETS camera_component
  DESTINATION lib)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
