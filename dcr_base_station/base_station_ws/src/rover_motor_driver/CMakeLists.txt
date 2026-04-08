cmake_minimum_required(VERSION 3.8)
project(rover_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

add_library(rover_motor_driver SHARED src/bld305s_hardware.cpp)
target_include_directories(rover_motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)
target_link_libraries(rover_motor_driver ${MODBUS_LIBRARIES})
ament_target_dependencies(rover_motor_driver
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface rover_motor_driver.xml)

install(TARGETS rover_motor_driver DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES rover_motor_driver.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
