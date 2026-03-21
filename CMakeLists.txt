cmake_minimum_required(VERSION 3.16)
project(roboclaw_hardware LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Service interfaces
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "srv/StopMotors.srv"
  "srv/ResetEncoders.srv"
  "srv/GetMotorStatus.srv"
  "srv/SetPIDGains.srv"
  "srv/ClearErrors.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

# ---------------------------------------------------------------------------
# Library: roboclaw_hardware
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/unit_converter.cpp
  src/roboclaw_tcp.cpp
  src/roboclaw_protocol.cpp
  src/roboclaw_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOCLAW_HARDWARE_BUILDING_DLL)

pluginlib_export_plugin_description_file(hardware_interface roboclaw_hardware_plugin.xml)

# ---------------------------------------------------------------------------
# Executable: roboclaw_service_node
# ---------------------------------------------------------------------------
add_executable(roboclaw_service_node src/roboclaw_service_node.cpp
  src/roboclaw_tcp.cpp
  src/roboclaw_protocol.cpp
)
target_include_directories(roboclaw_service_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(roboclaw_service_node rclcpp)
target_link_libraries(roboclaw_service_node "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME} roboclaw_service_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_unit_converter test/test_unit_converter.cpp)
  target_link_libraries(test_unit_converter ${PROJECT_NAME})
  target_include_directories(test_unit_converter PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_protocol_crc test/test_protocol_crc.cpp)
  target_link_libraries(test_protocol_crc ${PROJECT_NAME})
  target_include_directories(test_protocol_crc PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
