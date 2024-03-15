#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener() : Node("tf_listener")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =  std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer( 50ms, std::bind(&FrameListener::on_timer, this));

    odometry_trasformata = this->create_publisher<nav_msgs::msg::Odometry>("/odom_tf_translated", 10);
  }

private:
  void printTransformStamped(const geometry_msgs::msg::TransformStamped& transformStamped) {
    std::cout << "---\nHeader:" << std::endl;
    std::cout << "  Frame ID: " << transformStamped.header.frame_id << std::endl;
    std::cout << "  Stamp: " << transformStamped.header.stamp.sec << "s " << transformStamped.header.stamp.nanosec << "ns" << std::endl;
    std::cout << "Transform:" << std::endl;
    std::cout << "  Translation:" << std::endl;
    std::cout << "    x: " << transformStamped.transform.translation.x << std::endl;
    std::cout << "    y: " << transformStamped.transform.translation.y << std::endl;
    std::cout << "    z: " << transformStamped.transform.translation.z << std::endl;
    std::cout << "  Rotation:" << std::endl;
    std::cout << "    x: " << transformStamped.transform.rotation.x << std::endl;
    std::cout << "    y: " << transformStamped.transform.rotation.y << std::endl;
    std::cout << "    z: " << transformStamped.transform.rotation.z << std::endl;
    std::cout << "    w: " << transformStamped.transform.rotation.w << std::endl;

    nav_msgs::msg::Odometry odom;
    odom.header = transformStamped.header;
    odom.child_frame_id = transformStamped.child_frame_id;
    odom.pose.pose.position.x = transformStamped.transform.translation.x;
    odom.pose.pose.position.y = transformStamped.transform.translation.y;
    odom.pose.pose.position.z = transformStamped.transform.translation.z;
    odom.pose.pose.orientation.x = transformStamped.transform.rotation.x;
    odom.pose.pose.orientation.y = transformStamped.transform.rotation.y;
    odom.pose.pose.orientation.z = transformStamped.transform.rotation.z;
    odom.pose.pose.orientation.w = transformStamped.transform.rotation.w;
    odometry_trasformata->publish(odom);

  }


  void on_timer()
  {

    geometry_msgs::msg::TransformStamped t;

    try {
          t = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel,tf2::TimePointZero);
          printTransformStamped(t);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        }  
  }
  
  const std::string fromFrameRel = "orbslam";
  const std::string toFrameRel = "fl_track";

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_trasformata;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}