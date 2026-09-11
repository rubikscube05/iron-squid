cmake_minimum_required(VERSION 3.8)

project(octopus_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ============================================================
# INSTALL ROBOT FILES
# ============================================================

install(
  DIRECTORY
    launch
    worlds
    urdf
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# INSTALL PYTHON CONTROLLER
# ============================================================

install(
  PROGRAMS
    scripts/servo_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# TESTING
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
