#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace rclcpp;


int main(int argc, char *argv[])
{

  std::cout << "Inizio" << std::endl;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("publish_tf_orbslam");

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

  float RATE = 20.0;
  int tempo_sleep=(1/RATE)*1000;  // in millisecondi

  if (argc==3){
    tempo_sleep = atoi(argv[2]);
  }
  std::cout << "tempo_sleep: " << tempo_sleep << std::endl;

  auto tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

  // Lettura e pubblicazione
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double timestamp_double;
  
    Eigen::Quaternionf q;
    Eigen::Vector3f twc;
    std::istringstream ss(line);
    ss >> timestamp_double >> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w() ;
    
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

    sleep_for(std::chrono::milliseconds(tempo_sleep));

  }

  std::cout << "FINEE "  << std::endl;


  file.close();
  shutdown();
  return 0;
}
