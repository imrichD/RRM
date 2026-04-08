cmake_minimum_required(VERSION 3.8)
project(block2_dzvonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(model_spawner src/model_spawner.cpp)
ament_target_dependencies(model_spawner
  rclcpp
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  model_spawner
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Preskočíme niektoré lintery, ak ich nepoužívaš
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()