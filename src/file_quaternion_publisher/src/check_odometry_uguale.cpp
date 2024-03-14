// File: show_image/src/main.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <cstdlib>
#include "geometry_msgs/msg/transform_stamped.hpp"

double degreesToRadians(double degrees)
{
  return degrees * (M_PI / 180.0);
}

double first_x = -1, first_y = -1;

int main(int argc, char *argv[])
{
  std::cout << "Inizio" << std::endl;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odometry_publisher");

  if (argc <= 1)
  {
    std::cerr << "Usage: ros2 run file_quaternion_publisher check_odometry_uguale /path/to/file_with_quaternions_effettivi.txt  /path/to/file_with_quaternions_orbslam.txt [tempo_sleep]" << std::endl;
    std::cerr << "Primo file: quello che contiene l'odometry giusta. Secondo file: quello di orbslam. Entrambi sono nello stesso formato del file_quaternion_publisher.cpp.\n"
              << "Su RVIZ2: le frecce verdi sono quelle efettive, quelle rosse sono quelle di orbslam\n"
              << "\ttempo_sleep e' la distanza (in millisecondi) tra una pubblicazione di un'odometry e la successiva. Default: 100.' " << std::endl;
    return 1;
  }

  // Tempo sleep:
  int tempo_sleep = 10;
  if (argc == 4)
  {
    tempo_sleep = atoi(argv[3]);
  }
  std::cout << "tempo_sleep: " << tempo_sleep << std::endl;

  std::string file_path_orbslam = argv[2];
  std::ifstream file_orbslam(file_path_orbslam);

  // Open the file
  std::string file_path = argv[1];
  std::ifstream file(file_path);

  // Check if the file is opened successfully
  if (!file.is_open())
  {
    RCLCPP_ERROR(node->get_logger(), "Error opening file: %s", file_path.c_str());
    return 1;
  }

  // Due nodi per pubblicare il quaternione EFFETTIVO (VERDE) e quello di ORBSLAM (ROSSO)
  auto quaternion_pub_original = node->create_publisher<nav_msgs::msg::Odometry>("odom/original", 10);
  auto quaternion_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom/prova", 10);
  auto quaternion_prova = node->create_publisher<nav_msgs::msg::Odometry>("odom/castor", 10);

  system("rviz2 -d visualizza_odom.rviz  >/dev/null 2>/tmp/rviz_errors.txt  & ");
  rclcpp::sleep_for(std::chrono::milliseconds(5 * 1000));

  // FILE CON QUATERNIONI EFFETTIVI
  std::string line;
  int i = 0;
  Eigen::Vector3f twc_primo_val;
  // Read the file line by line
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    float togli_primo_valore;

    // Ricaviamo e pubblichiamo il quaternione originale di orbslam
    Eigen::Quaternionf q;
    Eigen::Vector3f twc;
    std::istringstream ss(line);
    ss >> togli_primo_valore >> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w();

    auto message = nav_msgs::msg::Odometry();
    message.header.frame_id = "track";

    geometry_msgs::msg::Pose output_pose{};

    if (i == 0)
    {
      first_x = twc.x();
      first_y = -twc.y();
    }

    output_pose.position.x = twc.x();
    output_pose.position.y = twc.y();
    output_pose.position.z = twc.z();

    output_pose.orientation.x = q.x();
    output_pose.orientation.y = q.y();
    output_pose.orientation.z = q.z();
    output_pose.orientation.w = q.w();

    message.pose.pose = output_pose;
    quaternion_pub_original->publish(message);

    rclcpp::sleep_for(std::chrono::milliseconds(tempo_sleep));
    i++;
  }

  // QUATERNIONI DI ORBSLAM
  while (std::getline(file_orbslam, line))
  {

    std::istringstream iss(line);
    float togli_primo_valore;

    Eigen::Quaternionf q;
    Eigen::Vector3f twc;
    std::istringstream ss(line);
    ss >> togli_primo_valore >> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w();

    auto message = nav_msgs::msg::Odometry();
    message.header.frame_id = "track";

    geometry_msgs::msg::Pose output_pose_ruotato{};

    output_pose_ruotato.position.x = sqrt(twc.z() * twc.z() + twc.y() * twc.y());

    output_pose_ruotato.position.x *= twc.z() < 0 ? -1 : 1;
    output_pose_ruotato.position.y = twc.x();
    output_pose_ruotato.position.z = 0;

    output_pose_ruotato.orientation.x = q.x();
    output_pose_ruotato.orientation.y = q.y();
    output_pose_ruotato.orientation.z = q.z();
    output_pose_ruotato.orientation.w = q.w();

    // Copiamo il quaternione
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(output_pose_ruotato.orientation, tf2_quat);

    // // Prendiamo roll, pitch e yaw del quaternione originale
    tf2::Matrix3x3 m(tf2_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // std::cout   << "\t" << roll << " " << pitch << " " << yaw << std::endl;

    // Ruotiamo e pubblichiamo il nuovo quaternione
    tf2_quat.setRPY(0, 0, yaw);

    output_pose_ruotato.orientation = tf2::toMsg(tf2_quat);
    message.pose.pose = output_pose_ruotato;
    quaternion_pub->publish(message);

    rclcpp::sleep_for(std::chrono::milliseconds(tempo_sleep));
  }

  // Close the file
  file.close();
  file_orbslam.close();
  // PROVA DI CASTOR

  std::ifstream fileread("matrice_orbslam3.txt");

  // Check if file is opened successfully
  if (!fileread.is_open())
  {
    std::cerr << "Error opening file." << std::endl;
    return 1;
  }

  // Initialize a cv::Mat for the 4x4 matrix
  cv::Mat pose(4, 4, CV_64F);

  // Read each line of the file
  
  while (std::getline(fileread, line))
  {
    // Create a string stream to parse the line
    std::istringstream iss(line);
    double value;
    // Read each value separated by whitespace and store them in the matrix
    for (int row = 0; row < 4; ++row)
    {
      for (int col = 0; col < 4; ++col)
      {
        if (!(iss >> value))
        {
          std::cerr << "Error reading value." << std::endl;
          return 1;
        }
        pose.at<double>(row, col) = value;
      }
    }

    // Transform logic
    if (pose.empty())
      return 0;

    static cv::Mat pose_prev = cv::Mat::eye(4, 4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4, 4, CV_32F);
    static const cv::Mat flipSign = (cv::Mat_<float>(4, 4) << 1, -1, -1, 1,
                                     -1, 1, -1, 1,
                                     -1, -1, 1, 1,
                                     1, 1, 1, 1);

    cv::Mat translation = (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    tf2::Matrix3x3 cameraRotation_rh(-world_lh.at<float>(0, 0), world_lh.at<float>(0, 1), world_lh.at<float>(0, 2),
                                     -world_lh.at<float>(1, 0), world_lh.at<float>(1, 1), world_lh.at<float>(1, 2),
                                     world_lh.at<float>(2, 0), -world_lh.at<float>(2, 1), -world_lh.at<float>(2, 2));

    tf2::Vector3 cameraTranslation_rh(world_lh.at<float>(0, 3), world_lh.at<float>(1, 3), -world_lh.at<float>(2, 3));

    const tf2::Matrix3x3 rotation270degXZ(0, 1, 0,
                                          0, 0, 1,
                                          1, 0, 0);

    tf2::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf2::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

    //static tf2_ros::TransformBroadcaster br;
    
    static tf2_ros::TransformBroadcaster br(node);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = node->now();
    transformStamped.header.frame_id = "camera_link";
    transformStamped.child_frame_id = "camera_pose";
    transformStamped.transform.translation.x = globalTranslation_rh.getX();
    transformStamped.transform.translation.y = globalTranslation_rh.getY();
    transformStamped.transform.translation.z = globalTranslation_rh.getZ();
    tf2::Quaternion q;
    globalRotation_rh.getRotation(q);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
  }
  
  fileread.close();

  std::string tmp;
  std::cout << "Inserisci qualcosa per terminare...";
  std::cin >> tmp;
  // std::cin.get();

  system("pkill rviz2");

  rclcpp::shutdown();
  return 0;
}