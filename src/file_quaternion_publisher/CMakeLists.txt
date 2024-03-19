cmake_minimum_required(VERSION 3.5)
project(file_quaternion_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Script da file a ROS2 
add_executable(file_quaternion_publisher src/file_quaternion_publisher.cpp)
target_include_directories(file_quaternion_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(file_quaternion_publisher rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)

install(TARGETS file_quaternion_publisher
  DESTINATION lib/${PROJECT_NAME})


# salva img da ros2 a file per calibrazione intrinsica
add_executable(salva_immagini src/salva_immagini.cpp)
target_include_directories(salva_immagini PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(salva_immagini rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)

install(TARGETS salva_immagini
  DESTINATION lib/${PROJECT_NAME})

# Da IMU come messaggio ROS2  a file csv
# add_executable(form_imu_msg_to_csv src/form_imu_msg_to_csv.cpp)
# target_include_directories(form_imu_msg_to_csv PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(form_imu_msg_to_csv rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)

# install(TARGETS form_imu_msg_to_csv
#   DESTINATION lib/${PROJECT_NAME})

# Estrattore da immagini e imu con ROS2 a file PNG e csv 
add_executable(from_ros2_images_imu_to_png_csv src/from_ros2_images_imu_to_png_csv.cpp)
target_include_directories(from_ros2_images_imu_to_png_csv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(from_ros2_images_imu_to_png_csv rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)

install(TARGETS from_ros2_images_imu_to_png_csv
  DESTINATION lib/${PROJECT_NAME})


# Check no perdita messaggi 
# add_executable(check_no_perdita_messaggi src/check_no_perdita_messaggi.cpp)
# target_include_directories(check_no_perdita_messaggi PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(check_no_perdita_messaggi rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)

#install(TARGETS check_no_perdita_messaggi
#  DESTINATION lib/${PROJECT_NAME})




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
