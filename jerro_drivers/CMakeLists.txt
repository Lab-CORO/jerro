cmake_minimum_required(VERSION 3.8)
project(jerro_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(jerro_msgs REQUIRED)
# find_package(wiringPi REQUIRED)

# Add include directories for header files
include_directories(include)


find_library(PIGPIO_LIB pigpio pigpiod_if2 wiringPi)
if(NOT PIGPIO_LIB)
  message(FATAL_ERROR "pigpio library not found. Please install it.")
endif()

SET(CMAKE_CXX_FLAGS "-fpermissive")

# Ajouter l'exécutable encodeurs
add_executable(encodeurs src/RED.cpp src/test_RED_dual.cpp)
ament_target_dependencies(encodeurs rclcpp std_msgs jerro_msgs)

target_link_libraries(encodeurs
  ${PIGPIO_LIB}
  pthread
  pigpiod_if2
  rt
)
install(TARGETS encodeurs
  DESTINATION lib/${PROJECT_NAME})


# central inertielle
add_executable(centrale_inertielle src/centrale_inertielle.cpp)
ament_target_dependencies(centrale_inertielle
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    geometry_msgs
    jerro_msgs
)
target_link_libraries(centrale_inertielle
  wiringPi
)
install(TARGETS centrale_inertielle
  DESTINATION lib/${PROJECT_NAME})



add_executable(servomotor src/servomoteur.cpp)
ament_target_dependencies(servomotor rclcpp jerro_msgs)

install(TARGETS
    servomotor
  DESTINATION lib/${PROJECT_NAME})


# Simple encoder publisher (replaces complex RED.cpp)
add_executable(encoder_publisher src/encoder_publisher.cpp)
ament_target_dependencies(encoder_publisher rclcpp std_msgs)
target_link_libraries(encoder_publisher
  ${PIGPIO_LIB}
  pthread
  pigpiod_if2
  rt
)
install(TARGETS encoder_publisher
  DESTINATION lib/${PROJECT_NAME})


# Motor PID controller with auto-tuning
add_executable(motor_pid_controller src/motor_pid_controller.cpp)
ament_target_dependencies(motor_pid_controller
  rclcpp
  rclcpp_action
  action_msgs
  std_msgs
  jerro_msgs
)
target_link_libraries(motor_pid_controller
  ${PIGPIO_LIB}
  pthread
  pigpiod_if2
  rt
)
install(TARGETS motor_pid_controller
  DESTINATION lib/${PROJECT_NAME})



# Install config, launch and scripts directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS  # Preserve execute permissions for scripts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
