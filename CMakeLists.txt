cmake_minimum_required(VERSION 3.16)
project(omnibot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  omnibot
  SHARED
  hardware/omnibot_system.cpp
)
target_compile_features(omnibot PUBLIC cxx_std_17)
target_include_directories(omnibot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/omnibot>
)
ament_target_dependencies(
  omnibot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "omnibot_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface omnibot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/omnibot
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/omnibot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/omnibot
)
install(TARGETS omnibot
  EXPORT export_omnibot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_omnibot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()