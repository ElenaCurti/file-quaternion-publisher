#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace rclcpp;

void shutdown_handler(int signal) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received signal: %d. Shutting down...", signal);
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{

  std::cout << "Inizio" << std::endl;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("publish_tf_orbslam");
      signal(SIGINT, shutdown_handler);


  // Check parametri
  if (argc <= 1) {
    std::cerr << "Usage: ros2 run file_quaternion_publisher publish_tf_orbslam /path/to/file_with_quaternions.txt [tempo_sleep]" << std::endl;
    std::cerr << "Il file da fornire e' quello restituito da orbslam alla fine dell'esecuzione (f_nome.txt). "<<  
                 "Deve contenere N righe con N=numero immagini processate. Ogni riga contiene le seguenti 8 colonne separate da spazi:" <<
                 "un valore che viene scartato,\n position x, position y, position z, quaternion x, quaternion y, quaternion z, quaternion w.   \n"<<
                 "\ttempo_sleep e' la distanza (in millisecondi) tra una pubblicazione di un'odometry e la successiva. Default: 100.' " << std::endl;
    return 1;
  }
  std::string file_path = argv[1];
  std::ifstream file(file_path);
  if (!file.is_open()) {
      RCLCPP_ERROR(node->get_logger(), "Error opening file: %s", file_path.c_str());
      return 1;
  }

  float RATE = 14.0;
  int tempo_sleep=(1/RATE)*1000;  // in millisecondi

  if (argc==3){
    tempo_sleep = atoi(argv[2]);
  }
  std::cout << "tempo_sleep: " << tempo_sleep << std::endl;

  auto tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  auto odometry_originale = node->create_publisher<nav_msgs::msg::Odometry>("/odom_orbslam", 10);
  

  // Lettura e pubblicazione
  std::string line;
  double old_timestamp_double = -1;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double timestamp_double;
  
    Eigen::Quaternionf q;
    Eigen::Vector3f twc;
    std::istringstream ss(line);
    ss >> timestamp_double >> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w() ;  // Simula chiamata di track orbslam
    
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = Time(timestamp_double);;
    cout << "->" << Time(timestamp_double).seconds() << "\t" << Time(timestamp_double).nanoseconds() << endl;;
    transform.header.frame_id = "fl_track";
    transform.child_frame_id = "orbslam";
    transform.transform.translation.x = twc.x();
    transform.transform.translation.y = twc.y();
    transform.transform.translation.z = twc.z();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(transform);
    tf_pub->publish(tf_msg);


    nav_msgs::msg::Odometry odom;
    odom.header = transform.header;
    odom.child_frame_id = transform.child_frame_id;
    odom.pose.pose.position.x = transform.transform.translation.x;
    odom.pose.pose.position.y = transform.transform.translation.y;
    odom.pose.pose.position.z = transform.transform.translation.z;
    odom.pose.pose.orientation.x = transform.transform.rotation.x;
    odom.pose.pose.orientation.y = transform.transform.rotation.y;
    odom.pose.pose.orientation.z = transform.transform.rotation.z;
    odom.pose.pose.orientation.w = transform.transform.rotation.w;
    odometry_originale->publish(odom);


    sleep_for(std::chrono::milliseconds(tempo_sleep));

  }

  std::cout << "FINEE "  << std::endl;


  file.close();
  shutdown();
  return 0;
}
