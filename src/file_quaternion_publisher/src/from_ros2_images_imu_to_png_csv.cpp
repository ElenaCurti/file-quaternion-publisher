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

/*
DALLA CARTELLA DELL'ESTRAZIONE
watch -n1 -c ' clear ; echo "LEFT IMAGES" && wc -l timestamp_foto_left.txt  && (ls -l left/ | wc -l ) && echo "" && echo "RIGHT IMAGES" && wc -l timestamp_foto_right.txt && (ls -l right/ | wc -l ) && echo "" && echo "IMU DATA" && wc -l imu_data.csv '
echo "LEFT IMAGES" && wc -l timestamp_foto_left.txt  && (ls -l left/ | wc -l ) && echo "" && echo "RIGHT IMAGES" && wc -l timestamp_foto_right.txt && (ls -l right/ | wc -l ) && echo "" && echo "IMU DATA" && wc -l imu_data.csv
*/

using namespace std::chrono_literals;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        std::cout <<"Inizio " << std::endl;
        
        // Subscribe camera
        subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/left/image_rect_color",
            10,
            std::bind(&ImageSubscriber::image1_callback, this, std::placeholders::_1));

        image_count_1 = 0;

        subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/right/image_rect_color",
            10,
            std::bind(&ImageSubscriber::image2_callback, this, std::placeholders::_1));

        image_count_2 = 0;

        cv_bridge_ = std::make_shared<cv_bridge::CvImage>();

        if (directory_exists(main_dir) || directory_exists(dir1) || directory_exists(dir2) ) {
            RCLCPP_ERROR(this->get_logger(), "Directory " + main_dir + " oppure " + dir1 +  " oppure " + dir2 +" gia' esistenti! Cancellarle e riprovare");
            exit(1);
        }

        if (!create_directory(main_dir) || !create_directory(dir1) || !create_directory(dir2))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directories for saving images.");
            exit(1);
        }

        // Salvo l'ordine delle foto
        ordine_foto_left.open(main_dir + "timestamp_foto_left.txt");
        if (!ordine_foto_left.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open timestamp_foto_left");
            exit(1);
        }

        ordine_foto_right.open(main_dir + "timestamp_foto_right.txt");
        if (!ordine_foto_right.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open timestamp_foto_right");
            exit(1);
        }


        // Subscription all'imu
        rclcpp::QoS qos(rclcpp::KeepLast(10));  // Keep the last 10 messages
        qos.best_effort();

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos, std::bind(&ImageSubscriber::imuCallback, this, std::placeholders::_1));

        imu_csv_data.open(main_dir+"imu_data.csv");
        if (!imu_csv_data.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open imu_csv_data file for writing");
            exit(1);
        }
        else
        {
            // Write header to CSV file
            imu_csv_data << "# Timestamp,Angular Velocity X,Angular Velocity Y,Angular Velocity Z,"
                         "Linear Acceleration X,Linear Acceleration Y,Linear Acceleration Z\n";
        }

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
        
        int64_t sec = msg->header.stamp.sec;
        int64_t nsec = msg->header.stamp.nanosec;
        int64_t timestamp = sec * 1000000000 + nsec;
        

        //std::string image_filename = main_dir + "/" + camera_name + "/" + sec + "." + nsec + ".png";
        std::string t = std::to_string(timestamp) ;
        std::string image_filename = dir1 + t+ ".png";
        ordine_foto_left << t << std::endl;
        ordine_foto_left.flush();
        cv::imwrite(image_filename, cv_bridge_->image);
        RCLCPP_INFO(this->get_logger(), "Saved left image: %s", image_filename.c_str());

        image_count_1 ++;
    }

     void image2_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
        
        int64_t sec = msg->header.stamp.sec;
        int64_t nsec = msg->header.stamp.nanosec;
        int64_t timestamp = sec * 1000000000 + nsec;
        

        //std::string image_filename = main_dir + "/" + camera_name + "/" + sec + "." + nsec + ".png";
        std::string t = std::to_string(timestamp) ;
        std::string image_filename = dir2 + t+ ".png";
        ordine_foto_right << t << std::endl;
        ordine_foto_right.flush();
        cv::imwrite(image_filename, cv_bridge_->image);
        RCLCPP_INFO(this->get_logger(), "Saved right image: %s", image_filename.c_str());

        image_count_2 ++;
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

    

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Write data to CSV file

        int64_t sec = msg->header.stamp.sec;
        int64_t nsec = msg->header.stamp.nanosec;
        int64_t timestamp = sec * 1000000000 + nsec;
        imu_csv_data << timestamp << ","
                  << msg->angular_velocity.x << "," << msg->angular_velocity.y << ","
                  << msg->angular_velocity.z << "," << msg->linear_acceleration.x << ","
                  << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "\n";
        imu_csv_data.flush();


        RCLCPP_INFO(this->get_logger(), "Saved imu %s", std::to_string(timestamp));

    }

    void handleSignal(int signal) {
        if (signal == SIGINT) {
            imu_csv_data.close();
            ordine_foto_left.close();
            ordine_foto_right.close();
            imu_csv_matching_timestamp.close();
            rclcpp::shutdown();
        }
    }



    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::ofstream imu_csv_data, imu_csv_matching_timestamp;
    std::ofstream ordine_foto_left, ordine_foto_right;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
    int image_count_1, image_count_2;
    // Create directories for saving images
    std::string main_dir = "/path/to/folder/"; // Path in cui salvare tutto. METTI LO / FINALE 
    std::string dir1 = main_dir+"left/";
    std::string dir2 = main_dir+"right/";
    //std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

};

int main(int argc, char *argv[])
{

    //signal(SIGINT, handleSignal);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}
