// File: show_image/src/main.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>


class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber(char* topic_name)
    : Node("image_subscriber")
  {
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        processImage(msg);
      });

    RCLCPP_INFO(get_logger(), "Image subscriber node has been initialized.");
  }

private:
  void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout <<"sono qui2" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    std::cout <<"sono qui" << std::endl;

    cv::imshow("Camera Image", cv_ptr->image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};




int main(int argc, char *argv[])
{
  std::cout <<"inizio" << std::endl;
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<ImageSubscriber>(argv[1]));
  auto node = std::make_shared<rclcpp::Node>("odometry_publisher");

  const std::string file_path = "/home/formula-student/f_amir_mono_veloce_backup.txt";

    // Open the file
    std::ifstream file(file_path);

    // Check if the file is opened successfully
    if (!file.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Error opening file: %s", file_path.c_str());
        return 1;
    }


    // Parse permutations from command-line arguments
    std::vector<std::vector<int>> permutations;
    for (int i = 1; i < argc; ++i) {
        std::vector<int> perm;
        std::istringstream iss(argv[i]);
        int val;
        while (iss >> val) {
            perm.push_back(val);
        }
        permutations.push_back(perm);
    }


    auto quaternion_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom/prova", 10);

    // Read the file line by line
    std::string line;
    std::cin.get();

    while (std::getline(file, line)) {
        // Use std::istringstream to separate values
        std::istringstream iss(line);
        


        auto message = nav_msgs::msg::Odometry();
        geometry_msgs::msg::Pose output_pose{};
        float togli_primo_valore;
        Eigen::Quaternionf q;
        Eigen::Vector3f twc;
        std::istringstream ss(line);
        ss >> togli_primo_valore>> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w() ;

        /*output_pose.orientation.x = q.x();
        output_pose.orientation.y = q.y();
        output_pose.orientation.z = q.z();
        output_pose.orientation.w = q.w();*/

        for (const auto& perm : permutations) {
          output_pose.orientation.x = perm[0] == 0 ? q.x() : (perm[0] == 1 ? q.y() : (perm[0] == 2 ? q.z() : q.w()));
          output_pose.orientation.y = perm[1] == 0 ? q.x() : (perm[1] == 1 ? q.y() : (perm[1] == 2 ? q.z() : q.w()));
          output_pose.orientation.z = perm[2] == 0 ? q.x() : (perm[2] == 1 ? q.y() : (perm[2] == 2 ? q.z() : q.w()));
          output_pose.orientation.w = perm[3] == 0 ? q.x() : (perm[3] == 1 ? q.y() : (perm[3] == 2 ? q.z() : q.w()));        
        }

        
        output_pose.position.x = twc(0);
        output_pose.position.y = twc(2);
        output_pose.position.z = 0 ;


        
      
        
        // message.data = std::to_string(timestamp) + " " + messaggio_quaternion + "\n";
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        

        /*std::string value2;
        int count = -1;
        while (iss >> value2) {
          float value = std::stof(value2);

          std::cout << count << " "<<value << "\t" ;

            switch(count){
              case -1:
                break;
              case 0:
                output_pose.position.x =  value;
                break;
              case 1:
                output_pose.position.y =  value ;
                break;
              case 2:
                output_pose.position.z = 0 ;
                break;
              case 3: 
                output_pose.orientation.x =  value;
                break;
              case 4: 
                output_pose.orientation.y =  value;
                break;
              case 5: 
                output_pose.orientation.z =  value;
                break;
              case 6: 
                output_pose.orientation.w =   value;
                break;

            }
            
            count ++;
        }

        std::cout << std::endl;*/

        message.pose.pose = output_pose;
        message.header.frame_id = "map";

        quaternion_pub->publish(message);


        

        // Sleep for a short duration (adjust as needed)
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    // Close the file
    file.close();


  rclcpp::shutdown();
  return 0;
}