cmake_minimum_required(VERSION 3.8)
project(kros_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Hardware plugin
add_library(kros_car_system SHARED
  src/kros_car_system_common.cpp
  src/kros_car_system_threads.cpp
  src/kros_car_system_lifecycle.cpp
  src/kros_car_system_io.cpp
  src/wheel_protocol.cpp
)

target_include_directories(kros_car_system PUBLIC
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_compile_features(kros_car_system PUBLIC cxx_std_17)

target_link_libraries(kros_car_system
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(kros_car_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface kros_car_hardware_plugins.xml)

# Controller plugin
add_library(kros_car_controllers SHARED
  src/wheel_velocity_controller.cpp
)

target_include_directories(kros_car_controllers PUBLIC
  include
)

target_compile_features(kros_car_controllers PUBLIC cxx_std_17)

ament_target_dependencies(kros_car_controllers
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface kros_car_controller_plugins.xml)

install(
  TARGETS kros_car_system kros_car_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES kros_car_hardware_plugins.xml kros_car_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheel_protocol
    test/test_wheel_protocol.cpp
    src/wheel_protocol.cpp
  )
  target_include_directories(test_wheel_protocol PRIVATE include)
  target_compile_features(test_wheel_protocol PRIVATE cxx_std_17)
endif()

ament_package()
