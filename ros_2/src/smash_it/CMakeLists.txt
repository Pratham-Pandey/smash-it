cmake_minimum_required(VERSION 3.16)
project(smash_it LANGUAGES CXX)

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
  smash_it
  SHARED
  hardware/smashit_system.cpp
)
target_compile_features(smash_it PUBLIC cxx_std_17)
target_include_directories(smash_it PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/smash_it>
)


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# For moving cart to a specific pose.
add_executable(play src/play.cpp)

ament_target_dependencies(play rclcpp std_msgs geometry_msgs)

ament_target_dependencies(
  smash_it PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface smash_it.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/smash_it
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/smash_it
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/smash_it
)
install(TARGETS smash_it
  EXPORT export_smash_it
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  play
  DESTINATION lib/${PROJECT_NAME})


## EXPORTS
ament_export_targets(export_smash_it HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
