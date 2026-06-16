cmake_minimum_required(VERSION 3.8)
project(bumperbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/pd_motion_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
  pd_motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
ament_package()
