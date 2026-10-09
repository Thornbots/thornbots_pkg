cmake_minimum_required(VERSION 3.8)
project(thornbots_pkg)

# All nodes are C++; only the launch file is Python.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dji_serial_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(armor_tracker STATIC src/armor_tracker.cpp)
target_compile_features(armor_tracker PUBLIC cxx_std_17)
target_include_directories(armor_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(armor_tracker PUBLIC Eigen3::Eigen)

add_executable(target_tracker src/target_tracker.cpp)
target_link_libraries(target_tracker armor_tracker)
ament_target_dependencies(target_tracker
  rclcpp dji_serial_bridge geometry_msgs std_msgs tf2 tf2_ros)

# Pure logic (no rclcpp), one small library per node, unit-tested standalone.
foreach(core target_selector_core mcb_relay_core point_to_cv_target_core)
  add_library(${core} STATIC src/${core}.cpp)
  target_compile_features(${core} PUBLIC cxx_std_17)
  target_include_directories(${core} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(${core} PUBLIC Eigen3::Eigen)
endforeach()

add_library(point_to_cv_target_node STATIC src/point_to_cv_target_node.cpp)
target_link_libraries(point_to_cv_target_node PUBLIC point_to_cv_target_core)
ament_target_dependencies(point_to_cv_target_node PUBLIC
  rclcpp dji_serial_bridge geometry_msgs std_msgs tf2 tf2_ros)

add_executable(target_selector src/target_selector.cpp)
target_link_libraries(target_selector target_selector_core)
ament_target_dependencies(target_selector rclcpp dji_serial_bridge geometry_msgs)

add_executable(mcb_relay src/mcb_relay.cpp)
target_link_libraries(mcb_relay mcb_relay_core)
ament_target_dependencies(mcb_relay rclcpp dji_serial_bridge geometry_msgs nav_msgs)

add_executable(point_to_cv_target src/point_to_cv_target.cpp)
target_link_libraries(point_to_cv_target point_to_cv_target_node)

add_executable(pose_translator src/pose_translator.cpp)
ament_target_dependencies(pose_translator rclcpp dji_serial_bridge nav_msgs sensor_msgs)

add_executable(lidar_self_filter src/lidar_self_filter.cpp)
ament_target_dependencies(lidar_self_filter rclcpp sensor_msgs)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS target_tracker target_selector mcb_relay point_to_cv_target pose_translator
  lidar_self_filter odom_tf_broadcaster DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.urdf.xacro")
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.stl")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_armor_tracker test/test_armor_tracker.cpp)
  target_link_libraries(test_armor_tracker armor_tracker)
  ament_add_gtest(test_target_selector_core test/test_target_selector_core.cpp)
  target_link_libraries(test_target_selector_core target_selector_core)
  ament_add_gtest(test_mcb_relay_core test/test_mcb_relay_core.cpp)
  target_link_libraries(test_mcb_relay_core mcb_relay_core)
  ament_add_gtest(test_point_to_cv_target_core test/test_point_to_cv_target_core.cpp)
  target_link_libraries(test_point_to_cv_target_core point_to_cv_target_core)
  ament_add_gtest(test_point_to_cv_target_node test/test_point_to_cv_target_node.cpp)
  target_link_libraries(test_point_to_cv_target_node point_to_cv_target_node)

  # Python is left for the ament lint checks (launch/, test/).
  find_package(ament_cmake_pytest REQUIRED)
  # The image's pip anyio ships a pytest plugin that breaks apt pytest; see setup.cfg.
  ament_add_pytest_test(thornbots_pkg_pytest test TIMEOUT 300
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
endif()

ament_package()
