cmake_minimum_required(VERSION 3.8)
project(anscer_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiMapNavigate.action"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)
add_executable(multi_map_node src/multi_map_node.cpp)

ament_target_dependencies(multi_map_node
  rclcpp
  rclcpp_action
  nav2_msgs
  tf2
  tf2_geometry_msgs
)
find_library(SQLITE3_LIB sqlite3)
target_link_libraries(multi_map_node ${SQLITE3_LIB})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(multi_map_node ${cpp_typesupport_target})
# add_dependencies(multi_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${cpp_typesupport_target})
# rosidl_target_interfaces(multi_map_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS multi_map_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
