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


using namespace std::chrono_literals;

class DataPublisherNode : public rclcpp::Node
{
public:
    DataPublisherNode(std::string file_path) : Node("data_publisher")
    {
        file_path_g = file_path;
        
        // Publisher for original odometry
        tf_pub = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_orbslam", 10);

        // Subscriber for the background odometry
        rclcpp::QoS fastlio_qos(rclcpp::KeepLast(10)); // Example QoS settings, adjust as needed
        fastlio_qos.best_effort();
        
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry/fastLioOdom", fastlio_qos, std::bind(&DataPublisherNode::backgroundOdometryCallback, this, std::placeholders::_1));

        // Read file
        readFileAndPublish();
    }

private:
    void readFileAndPublish()
    {
        std::ifstream file(file_path_g);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", file_path_g.c_str());
            return;
        }
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            // Publish the data if it's synchronized with the background odometry
            publishIfSynchronized(iss);

            // Sleep or wait for synchronization
            // (This will be handled inside publishIfSynchronized)
        }
    }

    void backgroundOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // You can add logic here if needed
        // This callback will be called whenever a new message is received on the background odometry topic
        ros_time_of_background_odometry = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        // cout << "nuova odometry" << endl;
    }

    void publishIfSynchronized(std::istringstream& iss)
    {
        // Implement logic to wait for synchronization with background odometry
        // For simplicity, let's assume a synchronization threshold of 0.1 seconds
        const double synchronization_threshold = 0.1;

      double timestamp_double;
      
        Eigen::Quaternionf q;
        Eigen::Vector3f twc;
        
        iss >> timestamp_double >> twc(0) >> twc(1) >> twc(2) >> q.x() >> q.y() >> q.z() >> q.w() ;  // Simula chiamata di track orbslam
        
        double timestamp_seconds = timestamp_double * 1e-9;

        // Check if the current timestamp is synchronized with the background odometry
        // If not, wait until synchronization is achieved
        while (ros_time_of_background_odometry < timestamp_seconds - synchronization_threshold) {
            // You may add a small sleep here to avoid busy waiting
            // std::this_thread::sleep_for(10ms);
            // cout << "Diff: " << (ros_time_of_background_odometry -timestamp_seconds)  << endl;
            rclcpp::spin_some(this->get_node_base_interface());
        }

        
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = Time(timestamp_double);;
        cout << "->" << Time(timestamp_double).seconds() << "\t" << Time(timestamp_double).nanoseconds() << endl;;
        transform.header.frame_id = "fl_track";
        transform.child_frame_id = "orbslam";
        transform.transform.translation.x = twc.z();
        transform.transform.translation.y = -twc.x();
        transform.transform.translation.z = 0;
        transform.transform.rotation.x = -q.z() ;
        transform.transform.rotation.y = -q.x();
        transform.transform.rotation.z = -q.y() ;
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
        odometry_publisher_->publish(odom);

    }

    string file_path_g;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    double ros_time_of_background_odometry = 0.0; // Update this variable when receiving background odometry
};

int main(int argc, char * argv[])
{

  // Check parametri
  if (argc <= 1) {
    std::cerr << "Usage: ros2 run file_quaternion_publisher publish_tf_orbslam /path/to/file_with_quaternions.txt [tempo_sleep]" << std::endl;
    std::cerr << "Il file da fornire e' quello restituito da orbslam alla fine dell'esecuzione (f_nome.txt). "<<  
                 "Deve contenere N righe con N=numero immagini processate. Ogni riga contiene le seguenti 8 colonne separate da spazi:" <<
                 "un valore che viene scartato,\n position x, position y, position z, quaternion x, quaternion y, quaternion z, quaternion w.   \n"<<
                 "\ttempo_sleep e' la distanza (in millisecondi) tra una pubblicazione di un'odometry e la successiva. Default: 100.' " << std::endl;
    return 1;
  }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataPublisherNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
