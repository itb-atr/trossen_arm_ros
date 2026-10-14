cmake_minimum_required(VERSION 3.20)
project(trossen_arm_utilities)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(trossen_arm_recovery_node
  src/trossen_arm_recovery_node.cpp
)

target_link_libraries(trossen_arm_recovery_node PUBLIC
  ${controller_manager_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
)

install(
  TARGETS trossen_arm_recovery_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
