cmake_minimum_required(VERSION 3.16)
project(psyonic_bring_up)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(msg_dexmobile REQUIRED)

# Detect platform
if(WIN32)
  add_definitions(-DPLATFORM_WINDOWS)
elseif(UNIX)
  add_definitions(-DPLATFORM_LINUX)
endif()

# Create the hardware interface library
add_library(
  psyonic_hardware_interface
  SHARED
  src/psyonic_hardware_interface.cpp
  src/api.cpp
  src/hand.cpp
  src/parser.cpp
  src/ppp.cpp
  src/serial_helper.cpp
  src/wrapper.cpp
)

# Add platform specific source files
if(WIN32)
  target_sources(psyonic_hardware_interface PRIVATE src/winserial.cpp)
elseif(UNIX)
  target_sources(psyonic_hardware_interface PRIVATE src/linux_serial.cpp)
endif()

target_compile_features(psyonic_hardware_interface PUBLIC cxx_std_17)
target_include_directories(psyonic_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  psyonic_hardware_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  msg_dexmobile
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(psyonic_hardware_interface PRIVATE "PSYONIC_BRING_UP_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface psyonic_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


install(TARGETS psyonic_hardware_interface
  EXPORT export_psyonic_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_psyonic_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()