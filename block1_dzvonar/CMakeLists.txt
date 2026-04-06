cmake_minimum_required(VERSION 3.8)
project(block1_dzvonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rrm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IkAllSolutions.srv"
  "srv/IkBestSolution.srv"
  "srv/MoveCartesian.srv"
  "srv/MovePose.srv"
  "srv/Ik6AllSolutions.srv"
  "srv/Ik6BestSolution.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
  geometry_msgs
  tf2_ros
)

add_executable(ik_solver_node
  src/ik_solver_node.cpp
)
target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ik_solver_node PUBLIC cxx_std_17)
ament_target_dependencies(ik_solver_node
  rclcpp
)
target_link_libraries(ik_solver_node
  ${cpp_typesupport_target}
)

add_executable(motion_manager_node
  src/motion_manager_node.cpp
)
target_include_directories(motion_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(motion_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(motion_manager_node
  rclcpp
  sensor_msgs
  rrm_msgs
)
target_link_libraries(motion_manager_node
  ${cpp_typesupport_target}
)

add_executable(cartesian_control_client
  src/cartesian_control_client.cpp
)
target_include_directories(cartesian_control_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_compile_features(cartesian_control_client PUBLIC cxx_std_17)
ament_target_dependencies(cartesian_control_client
  rclcpp
)
add_dependencies(cartesian_control_client ${PROJECT_NAME})
target_link_libraries(cartesian_control_client
  ${cpp_typesupport_target}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS logger_node ik_solver_node motion_manager_node cartesian_control_client
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()