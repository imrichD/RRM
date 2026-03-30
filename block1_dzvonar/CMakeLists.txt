cmake_minimum_required(VERSION 3.8)
project(block1_dzvonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rrm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dzvonar_interface REQUIRED)
find_package(Eigen3 REQUIRED)

# trajectory_io as shared library so both nodes can use it
add_library(trajectory_io src/trajectory_io.cpp)
target_include_directories(trajectory_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(trajectory_io PUBLIC cxx_std_17)


add_executable(teleop_node src/teleop_node.cpp)
add_executable(logger_node src/logger_node.cpp)

target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(logger_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>${EIGEN3_INCLUDE_DIRS})

target_compile_features(teleop_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(logger_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(teleop_node "rclcpp" "rrm_msgs" "dzvonar_interface")
ament_target_dependencies(logger_node "rclcpp" "sensor_msgs" "dzvonar_interface" "rrm_msgs")

target_link_libraries(logger_node trajectory_io)
target_link_libraries(teleop_node trajectory_io)

install(TARGETS teleop_node logger_node trajectory_io
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()