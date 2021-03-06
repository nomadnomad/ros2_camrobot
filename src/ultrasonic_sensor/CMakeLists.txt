cmake_minimum_required(VERSION 3.5)
project(ultrasonic_sensor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ultrasonic_sensor_msgs REQUIRED)

add_executable(ultrasonic_sensor
  src/ultrasonic_sensor.cpp
  src/ultrasonic_sensor_main.cpp
)

ament_target_dependencies(ultrasonic_sensor
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  std_msgs
  ultrasonic_sensor_msgs
)

install(TARGETS
  ultrasonic_sensor
  DESTINATION lib/${PROJECT_NAME}
)

target_compile_options(ultrasonic_sensor
  PUBLIC -Wall -pthread
)
target_link_libraries(ultrasonic_sensor
  pigpiod_if2
)

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

ament_package()
