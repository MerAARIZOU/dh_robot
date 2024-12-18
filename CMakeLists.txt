cmake_minimum_required(VERSION 3.8)
project(dh_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(programme1 src/programme1.cpp)
ament_target_dependencies(
   programme1
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

add_executable(programme2 src/programme2.cpp)
ament_target_dependencies(
   programme2
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

add_executable(programme3 src/programme3.cpp)
ament_target_dependencies(
   programme3
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

install(TARGETS
   programme1
   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
   programme2
   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
   programme3
   DESTINATION lib/${PROJECT_NAME})


install (
  DIRECTORY launch rviz urdf src
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
