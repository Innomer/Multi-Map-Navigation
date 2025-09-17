cmake_minimum_required(VERSION 3.8)
project(anscer_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveWormhole.srv"
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
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)
add_executable(wormhole_saver_service src/wormhole_saver_service.cpp)

ament_target_dependencies(wormhole_saver_service
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_library(SQLITE3_LIB sqlite3)
target_link_libraries(wormhole_saver_service ${SQLITE3_LIB})
rosidl_target_interfaces(wormhole_saver_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  wormhole_saver_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
