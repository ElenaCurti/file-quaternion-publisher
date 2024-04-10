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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(tf2_ros REQUIRED)  
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)


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

# Estrattore da immagini COMPRESSE e imu con ROS2 a file PNG e csv 
add_executable(from_ros2_images_compressed_imu_to_png_csv src/from_ros2_images_compressed_imu_to_png_csv.cpp)
target_include_directories(from_ros2_images_compressed_imu_to_png_csv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(from_ros2_images_compressed_imu_to_png_csv rclcpp sensor_msgs cv_bridge OpenCV nav_msgs image_transport)
install(TARGETS from_ros2_images_compressed_imu_to_png_csv
  DESTINATION lib/${PROJECT_NAME})

# Check odometry uguale 
# add_executable(check_odometry_uguale src/check_odometry_uguale.cpp)
# target_include_directories(check_odometry_uguale PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(check_odometry_uguale rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)
# install(TARGETS check_odometry_uguale
#   DESTINATION lib/${PROJECT_NAME})

# Estrattore odometry
# add_executable(odometry_to_file src/odometry_to_file.cpp)
# target_include_directories(odometry_to_file PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(odometry_to_file rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)
# install(TARGETS odometry_to_file
#   DESTINATION lib/${PROJECT_NAME})
  

# Pubblicare orbslam pose as tf 
add_executable(publish_tf_transform src/publish_tf_transform.cpp)
target_include_directories(publish_tf_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(publish_tf_transform rclcpp sensor_msgs cv_bridge OpenCV nav_msgs tf2_ros tf2_geometry_msgs)
install(TARGETS publish_tf_transform
  DESTINATION lib/${PROJECT_NAME})

  
# Tf listener
add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(
  tf_listener
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    nav_msgs
)

install(TARGETS
tf_listener
    DESTINATION lib/${PROJECT_NAME})




#---------------------------- Convertitore tf ----------------------------


# add_executable(tf_converter src/tf_converter.cpp)
# target_include_directories(tf_converter PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(tf_converter 
#   rclcpp sensor_msgs cv_bridge OpenCV nav_msgs tf2_ros tf2_geometry_msgs)

# find_library(TF2_LIBRARY tf2)
# if(NOT TF2_LIBRARY)
#     message(FATAL_ERROR "tf2 library not found")
# endif()

# find_library(TF2_ROS_LIBRARY tf2_ros)
# if(NOT TF2_ROS_LIBRARY)
#     message(FATAL_ERROR "tf2_ros library not found")
# endif()

# find_library(TF2_GEOMETRY_MSGS_LIBRARY tf2_geometry_msgs)
# if(NOT TF2_GEOMETRY_MSGS_LIBRARY)
#     message(FATAL_ERROR "tf2_geometry_msgs library not found")
# endif()

# target_link_libraries(tf_converter
#   ${TF2_LIBRARY}
#   ${TF2_ROS_LIBRARY}
#   ${TF2_GEOMETRY_MSGS_LIBRARY})
  
# install(TARGETS tf_converter
#   DESTINATION lib/${PROJECT_NAME})

# # Pubblica tempo
# add_executable(publish_time src/publish_time.cpp)
# target_include_directories(publish_time PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(publish_time rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)
# install(TARGETS publish_time
#   DESTINATION lib/${PROJECT_NAME})



# # Mostra immagini
# add_executable(show_images src/show_images.cpp)
# target_include_directories(show_images PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(show_images rclcpp sensor_msgs cv_bridge OpenCV nav_msgs)
# install(TARGETS show_images
#   DESTINATION lib/${PROJECT_NAME})


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
