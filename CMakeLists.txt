cmake_minimum_required(VERSION 3.16)
project(ackdrive_px4_mrover LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ackdrive_px4_mrover
  SHARED
  hardware/carlikebot_system.cpp
  hardware/main.cpp
)
add_executable(mrover_comms hardware/main.cpp)


target_compile_features(ackdrive_px4_mrover PUBLIC cxx_std_17)
target_include_directories(ackdrive_px4_mrover PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ackdrive_px4_mrover>
)
ament_target_dependencies(
  ackdrive_px4_mrover PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  std_msgs
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(mrover_comms rclcpp std_msgs)

target_sources(mrover_comms PRIVATE hardware/main.cpp)

target_link_libraries(ackdrive_px4_mrover PUBLIC
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ackdrive_px4_mrover.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ackdrive_px4_mrover
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ackdrive_px4_mrover
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ackdrive_px4_mrover
)
install(TARGETS ackdrive_px4_mrover
  EXPORT export_ackdrive_px4_mrover
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
    mrover_comms
    DESTINATION lib/ackdrive_px4_mrover)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_11_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_11_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_11_launch test/test_carlikebot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ackdrive_px4_mrover HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
