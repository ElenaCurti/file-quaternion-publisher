// File: show_image/src/main.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

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

    auto quaternion_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom/prova", 10);

    // Read the file line by line
    std::cout << "Apri rviz2 e sottoscriviti a odom/prova. Il frame_id e' map. Poi premi un tasto qui per continuare..." << std::endl;
    std::string line;
    std::cin.get();

    while (std::getline(file, line)) {
        // Use std::istringstream to separate values
        std::istringstream iss(line);
      
        auto message = nav_msgs::msg::Odometry();
        message.header.frame_id = "map";


        float togli_primo_valore;

        Eigen::Quaternionf q;
        Eigen::Vector3f twc;
        std::istringstream ss(line);
        ss >> togli_primo_valore>> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w() ;

        geometry_msgs::msg::Pose output_pose{};

        output_pose.orientation.x = q.x();
        output_pose.orientation.y = q.y();
        output_pose.orientation.z = q.z();
        output_pose.orientation.w = q.w();

        output_pose.position.x = twc(0);
        output_pose.position.y = twc(2);
        output_pose.position.z = 0 ;

        message.pose.pose = output_pose;
        
        quaternion_pub->publish(message);


        

        // Sleep for a short duration (adjust as needed)
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    // Close the file
    file.close();


  rclcpp::shutdown();
  return 0;
}