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
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(trajectory_io SHARED
  src/trajectory_io.cpp
)
target_include_directories(trajectory_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(trajectory_io PUBLIC cxx_std_17)

add_executable(teleop_node
  src/teleop_node.cpp
)
target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(teleop_node PUBLIC cxx_std_17)
ament_target_dependencies(teleop_node
  rclcpp
  rrm_msgs
  dzvonar_interface
)
target_link_libraries(teleop_node trajectory_io)

add_executable(logger_node
  src/logger_node.cpp
)
target_include_directories(logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(logger_node PUBLIC cxx_std_17)
ament_target_dependencies(logger_node
  rclcpp
  sensor_msgs
  dzvonar_interface
  geometry_msgs
  tf2_ros
)
target_link_libraries(logger_node
  
    trajectory_io
    Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS trajectory_io teleop_node logger_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()