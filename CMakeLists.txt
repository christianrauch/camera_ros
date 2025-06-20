cmake_minimum_required(VERSION 3.18)
project(camera_ros)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)

# find dependencies
set(PACKAGE_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "camera_info_manager"
  "cv_bridge"
)
foreach(PKGDEP IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKGDEP} REQUIRED)
endforeach()
pkg_check_modules(libcamera REQUIRED libcamera>=0.1)

# new param callbacks need at least 17.0.0
if(${rclcpp_VERSION} VERSION_GREATER_EQUAL "17")
    add_compile_definitions(RCLCPP_HAS_PARAM_EXT_CB)
endif()

# library with common utility functions for type conversions
add_library(utils OBJECT
  src/format_mapping.cpp
  src/pretty_print.cpp
)
target_include_directories(utils PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(utils ${libcamera_LINK_LIBRARIES})
target_link_libraries(
  utils
  ${sensor_msgs_TARGETS}
)
set_property(TARGET utils PROPERTY POSITION_INDEPENDENT_CODE ON)

# library for parameter/controls handling and conversion
add_library(param OBJECT
    src/clamp.cpp
    src/cv_to_pv.cpp
    src/pv_to_cv.cpp
    src/types.cpp
    src/type_extent.cpp
    src/ParameterHandler.cpp
    src/ParameterConflictHandler.cpp
)
target_include_directories(param PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(param ${libcamera_LINK_LIBRARIES})
target_link_libraries(
  param
  rclcpp::rclcpp
)
set_property(TARGET param PROPERTY POSITION_INDEPENDENT_CODE ON)

# composable ROS2 node
add_library(camera_component SHARED src/CameraNode.cpp)
rclcpp_components_register_node(camera_component PLUGIN "camera::CameraNode" EXECUTABLE "camera_node")
target_link_libraries(camera_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  camera_info_manager::camera_info_manager
  cv_bridge::cv_bridge
)
target_include_directories(camera_component PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(camera_component PUBLIC ${libcamera_LINK_LIBRARIES})
target_link_libraries(camera_component PRIVATE utils param)

ament_export_targets(camera_componentTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

install(TARGETS camera_component
  EXPORT camera_componentTargets
  DESTINATION lib)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.flake8")
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
