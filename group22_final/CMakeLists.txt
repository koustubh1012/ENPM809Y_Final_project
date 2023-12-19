cmake_minimum_required(VERSION 3.8)
project(group22_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(mage_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(ros2_aruco_interfaces REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(mage_msgs REQUIRED)


include_directories(include/group22_final)

install(DIRECTORY
config
launch
DESTINATION share/${PROJECT_NAME}/
)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs rclcpp_action rclcpp_components)# mage_msgs)

install(TARGETS 
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(battery_tf_broadcaster src/battery_tf_broadcaster.cpp)
ament_target_dependencies(battery_tf_broadcaster rclcpp rclcpp tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

install(TARGETS 
  battery_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
