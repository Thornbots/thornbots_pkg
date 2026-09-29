cmake_minimum_required(VERSION 3.8)
project(thornbots_pkg)

# C++ for target_tracker, Python for the other nodes, one package.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dji_serial_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
install(TARGETS target_tracker DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
file(GLOB PYTHON_NODES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*)
install(PROGRAMS ${PYTHON_NODES} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.urdf.xacro")
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.stl")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_armor_tracker test/test_armor_tracker.cpp)
  target_link_libraries(test_armor_tracker armor_tracker)

  find_package(ament_cmake_pytest REQUIRED)
  # The image's pip anyio ships a pytest plugin that breaks apt pytest; see setup.cfg.
  ament_add_pytest_test(thornbots_pkg_pytest test TIMEOUT 300
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
endif()

ament_package()
