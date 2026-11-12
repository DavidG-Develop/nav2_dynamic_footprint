cmake_minimum_required(VERSION 3.16)
project(nav2_dynamic_footprint LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(bondcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(geometric_shapes REQUIRED)

add_library(footprint_geometry SHARED src/geometry.cpp)
target_include_directories(footprint_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(dynamic_footprint SHARED src/dynamic_footprint.cpp)
target_link_libraries(dynamic_footprint footprint_geometry)
ament_target_dependencies(dynamic_footprint
  rclcpp rclcpp_lifecycle rcl_interfaces bondcpp rclcpp_components geometry_msgs std_msgs tf2 tf2_ros urdf geometric_shapes)
rclcpp_components_register_node(dynamic_footprint
  PLUGIN "nav2_dynamic_footprint::DynamicFootprint"
  EXECUTABLE dynamic_footprint_node)
rclcpp_components_register_node(dynamic_footprint
  PLUGIN "nav2_dynamic_footprint::DynamicFootprintLifecycle"
  EXECUTABLE dynamic_footprint_lifecycle_node)

install(TARGETS footprint_geometry dynamic_footprint
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle rcl_interfaces bondcpp geometry_msgs std_msgs tf2 tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry footprint_geometry)
  ament_add_gtest(test_collisions test/test_collisions.cpp)
  target_link_libraries(test_collisions dynamic_footprint)
  ament_target_dependencies(test_collisions rclcpp rclcpp_lifecycle rcl_interfaces bondcpp geometry_msgs std_msgs tf2 tf2_ros)
  target_compile_definitions(test_collisions PRIVATE
    TEST_MESH="${CMAKE_CURRENT_SOURCE_DIR}/test/tetrahedron.stl")
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(test_parameters_lifecycle test/test_parameters_lifecycle.cpp)
  target_link_libraries(test_parameters_lifecycle dynamic_footprint)
  ament_target_dependencies(test_parameters_lifecycle
    rclcpp rclcpp_lifecycle rcl_interfaces bondcpp geometry_msgs std_msgs tf2 tf2_ros lifecycle_msgs)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_integration.py TIMEOUT 90)
  add_launch_test(test/test_launch_modes.py TIMEOUT 120)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_flake8()
  ament_lint_cmake()
  ament_uncrustify()
endif()
ament_package()
