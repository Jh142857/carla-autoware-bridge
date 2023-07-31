cmake_minimum_required(VERSION 3.14)
project(carla_autoware_bridge)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(autoware_cmake REQUIRED)
autoware_package()
# find dependencies
#find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
#find_package(rclcpp REQUIRED)
#find_package(component_interface_specs REQUIRED)
#find_package(component_interface_utils REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

ament_auto_add_executable(carla_autoware_bridge src/carla_autoware_bridge.cpp)
#add_executable(carla_pose_initializer src/carla_pose_initializer.cpp)
ament_target_dependencies(carla_autoware_bridge carla_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        )

#install(TARGETS
#        carla_pose_initializer
#        DESTINATION lib/${PROJECT_NAME}
#        )
#
#ament_package()

