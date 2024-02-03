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
#include <cstdlib>

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}


int main(int argc, char *argv[])
{
  std::cout << "Inizio" << std::endl;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odometry_publisher");

  if (argc <= 1) {
    std::cerr << "Usage: ros2 run file_quaternion_publisher file_quaternion_publisher /path/to/file_with_quaternions.txt" << std::endl;
    std::cerr << "Il file da fornire e' quello restituito da orbslam alla fine dell'esecuzione (f_nome.txt). "<<  
                 "Deve contenere N righe con N=numero immagini processate. Ogni riga contiene le seguenti 8 colonne separate da spazi:" <<
                 "un valore che viene scartato,\n position x, position y, position z, quaternion x, quaternion y, quaternion z, quaternion w.   \n" << std::endl;
    return 1;
  }

  std::string file_path = argv[1];

  // Open the file
  std::ifstream file(file_path);

  // Check if the file is opened successfully
  if (!file.is_open()) {
      RCLCPP_ERROR(node->get_logger(), "Error opening file: %s", file_path.c_str());
      return 1;
  }

  // Due nodi per pubblicare il quaternione originale di orbslam (in verde) e quello da noi ruotato (in rosso)
  auto quaternion_pub_original = node->create_publisher<nav_msgs::msg::Odometry>("odom/original", 10);
  auto quaternion_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom/prova", 10);

  system("rviz2 -d visualizza_odom.rviz  >/dev/null 2>/dev/null  & ");
  rclcpp::sleep_for(std::chrono::milliseconds(5*1000));

  std::string line;
  // Read the file line by line
  while (std::getline(file, line)) {
    // Use std::istringstream to separate values
    std::istringstream iss(line);
    float togli_primo_valore;
  
    // Ricaviamo e pubblichiamo il quaternione originale di orbslam
    Eigen::Quaternionf q;
    Eigen::Vector3f twc;
    std::istringstream ss(line);
    ss >> togli_primo_valore>> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w() ;

    auto message = nav_msgs::msg::Odometry();
    message.header.frame_id = "map";        

    geometry_msgs::msg::Pose output_pose{};

    output_pose.position.x = twc(0);
    output_pose.position.y = twc(2);
    output_pose.position.z = 0 ;

    output_pose.orientation.x = q.x();
    output_pose.orientation.y = q.y();
    output_pose.orientation.z = q.z();
    output_pose.orientation.w = q.w();

    message.pose.pose = output_pose;    
    quaternion_pub_original->publish(message);


    // Nuovo messaggio: ci copiamo la position
    auto message2 = nav_msgs::msg::Odometry();
    message2.header.frame_id = "map";

    geometry_msgs::msg::Pose output_pose_ruotato{}; 
    output_pose_ruotato.position.x = twc(0);
    output_pose_ruotato.position.y = twc(2);
    output_pose_ruotato.position.z = 0 ;

    
    // Copiamo il quaternione
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(output_pose.orientation, tf2_quat);
    //std::cout   << "\t" << output_pose.orientation.x << " " <<  output_pose.orientation.y << " " <<  output_pose.orientation.z  << " " <<  output_pose.orientation.w  << std::endl;
    //std::cout   << "\t" << tf2_quat_from_msg.getX() << " " <<  tf2_quat_from_msg.getY() << " " <<  tf2_quat_from_msg.getZ()  << " " <<  tf2_quat_from_msg.getW()  << std::endl;
    
    // Prendiamo roll, pitch e yaw del quaternione originale
    tf2::Matrix3x3 m(tf2_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout   << "\t" << roll << " " << pitch << " " << yaw << std::endl;

    // Ruotiamo e pubblichiamo il nuovo quaternione
    tf2_quat.setRPY( 0.0, 0.0, yaw+degreesToRadians(90));

    output_pose_ruotato.orientation = tf2::toMsg(tf2_quat);
    message2.pose.pose = output_pose_ruotato;
    quaternion_pub->publish(message2);

    // Sleep for a short duration (adjust as needed)
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  // Close the file
  file.close();


  std::string tmp;
  std::cout << "Inserisci qualcosa per terminare..."; 
  std::cin >>  tmp;
  //std::cin.get();

  system("pkill rviz2");


  rclcpp::shutdown();
  return 0;
}