cmake_minimum_required(VERSION 3.5)
project(camera_basic_pkg)

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
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Build
add_executable(lanefollowing src/lanefollowing.cpp)
target_include_directories(lanefollowing PUBLIC ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
ament_target_dependencies(lanefollowing rclcpp cv_bridge sensor_msgs geometry_msgs OpenCV ackermann_msgs visualization_msgs nav_msgs)

#Build
add_executable(lanefollowing_intel src/lanefollowing_intel.cpp)
target_include_directories(lanefollowing_intel PUBLIC ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
ament_target_dependencies(lanefollowing_intel rclcpp cv_bridge sensor_msgs geometry_msgs OpenCV ackermann_msgs visualization_msgs nav_msgs)

#Install
install(TARGETS
  lanefollowing
  lanefollowing_intel
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
