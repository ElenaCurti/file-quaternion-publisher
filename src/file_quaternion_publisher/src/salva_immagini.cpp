#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <ctime>
#include <sys/stat.h>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <sensor_msgs/msg/imu.hpp>
#include <fstream>
#include <unistd.h>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        // Subscribe to the image topic
       
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/left_image",
            10,
            std::bind(&ImageSubscriber::image1_callback, this, std::placeholders::_1));

        // check if dir exists
        if (directory_exists(main_dir))
        {
            RCLCPP_ERROR(this->get_logger(), "Directory " + main_dir + " gia' esistenti! Cancellarle e riprovare");
            exit(1);
        }
 cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
        // check if i can create directory
        if (!create_directory(main_dir))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directories for saving images.");
            exit(1);
        }
        // Initialize OpenCV window
        cv::namedWindow("Left Image");
        /*cv::setMouseCallback(
            "Left Image", [](int event, int x, int y, int flags, void *userdata) {}, nullptr);*/

        // Initialize counter
        counter_ = 0;

        // Wait for spacebar input
        cv::waitKey(0);
    }

private:
    void image1_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow("Image Viewer", cv_bridge_->image);
    cv::waitKey(1);
        //std::string image_filename = main_dir + "/" + camera_name + "/" + sec + "." + nsec + ".png";
        
        //cv::imwrite(image_filename, cv_bridge_->image);
        RCLCPP_INFO(this->get_logger(), "Saved left image:");

        //image_count_1 ++;
    }

    bool create_directory(const std::string &dir)
    {
        if (mkdir(dir.c_str(), 0777) == -1)
        {
            return false;
        }
        return true;
    }

    bool directory_exists(const std::string &dir)
    {
        struct stat info;
        return stat(dir.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int counter_;

    const std::string main_dir = "left_image";
    
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
};

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}