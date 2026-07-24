cmake_minimum_required(VERSION 3.8)
project(bumperbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp CONFIG REQUIRED)

include_directories(include)

add_library(pd_motion_planner SHARED src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner
  rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core
  nav2_costmap_2d nav2_util pluginlib)

add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit
  rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core
  nav2_costmap_2d nav2_util pluginlib)

add_library(mpc_controller SHARED
  src/mpc_controller.cpp
  src/mpc_solver.cpp
  src/reference_trajectory.cpp
)
ament_target_dependencies(mpc_controller
  rclcpp geometry_msgs nav_msgs std_msgs tf2_ros tf2_geometry_msgs nav2_core
  nav2_costmap_2d nav2_util pluginlib osqp_vendor
)
target_link_libraries(mpc_controller Eigen3::Eigen osqp::osqp)

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/pd_motion_planner.py
  ${PROJECT_NAME}/pure_pursuit.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)


install(TARGETS
  pd_motion_planner
  pure_pursuit
  mpc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mpc_solver test/test_mpc_solver.cpp src/mpc_solver.cpp)
  target_link_libraries(test_mpc_solver Eigen3::Eigen osqp::osqp)
  ament_target_dependencies(test_mpc_solver geometry_msgs nav_msgs osqp_vendor)
  ament_add_gtest(test_reference_trajectory
    test/test_reference_trajectory.cpp src/reference_trajectory.cpp)
  ament_target_dependencies(test_reference_trajectory geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
  ament_add_gtest(test_mpc_controller test/test_mpc_controller.cpp)
  target_link_libraries(test_mpc_controller mpc_controller)
  ament_target_dependencies(test_mpc_controller
    rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2_ros nav2_core nav2_costmap_2d)
  ament_add_gtest(test_mpc_collision test/test_mpc_collision.cpp)
  target_link_libraries(test_mpc_collision mpc_controller)
  ament_target_dependencies(test_mpc_collision geometry_msgs nav2_core nav2_costmap_2d)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core
  nav2_costmap_2d nav2_util pluginlib Eigen3 osqp_vendor std_msgs)
ament_export_libraries(pd_motion_planner pure_pursuit mpc_controller)
ament_package()
