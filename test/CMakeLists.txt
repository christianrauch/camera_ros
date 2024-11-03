find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_test_param test_param.cpp)
target_include_directories(${PROJECT_NAME}_test_param PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_test_param
    rclcpp
    rclcpp_components
)

# link the 'camera_component' via '--no-as-needed' to find all plugins automatically via the ClassLoader
target_link_libraries(${PROJECT_NAME}_test_param "-Wl,--no-as-needed" camera_component)
