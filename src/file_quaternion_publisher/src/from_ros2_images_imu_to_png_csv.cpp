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



using namespace std::chrono_literals;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        std::cout <<"Inizio " << std::endl;
        
        // Subscribe camera
        subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/left_image",
            10,
            std::bind(&ImageSubscriber::image1_callback, this, std::placeholders::_1));

        image_count_1 = 0;

        cv_bridge_ = std::make_shared<cv_bridge::CvImage>();

        if (directory_exists(main_dir) || directory_exists(dir1) ) {
            RCLCPP_ERROR(this->get_logger(), "Directory " + main_dir + " oppure " + dir1 + " gia' esistenti! Cancellarle e riprovare");
            exit(1);
        }

        if (!create_directory(main_dir) || !create_directory(dir1) )
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directories for saving images.");
            exit(1);
        }

        // Salvo l'ordine delle foto
        ordine_foto.open(main_dir + "timestamp_foto.txt");
        if (!ordine_foto.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open timestamp_foto");
            exit(1);
        }


        // Subscription all'imu
        rclcpp::QoS qos(rclcpp::KeepLast(10));  // Keep the last 10 messages
        qos.best_effort();

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos, std::bind(&ImageSubscriber::imuCallback, this, std::placeholders::_1));

        imu_csv_data.open(main_dir+"imu_mapping_bag_parcheggio_orario.csv");
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

        imu_csv_matching_timestamp.open(main_dir+"imu_matching_original_timestamp.csv");
        if (!imu_csv_matching_timestamp.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open imu_csv_matching_timestamp file for writing");
            exit(1);
        }
        else
        {
            // Write header to CSV file
            imu_csv_matching_timestamp << "# Timestamp originale, Timestamp modificato\n";
        }


    }

private:


    std::string get_timestamp(){
        rclcpp::Time t = this->get_clock()->now();
        return std::to_string(t.nanoseconds()) ; //+ std::to_string(t.nanoseconds());

        
        // // Get the current time point using high_resolution_clock
        // auto currentTimePoint = std::chrono::high_resolution_clock::now();

        // // Convert the time point to a duration since the epoch
        // auto durationSinceEpoch = currentTimePoint.time_since_epoch();

        // // Extract the count of seconds and nanoseconds
        // auto seconds = std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch);
        // auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(durationSinceEpoch - seconds);

        // // Print the current timestamp in seconds and nanoseconds
        // //std::cout << "Current timestamp: " << seconds.count() << " seconds, " << nanoseconds.count() << " nanoseconds" << std::endl;
        // return std::to_string(seconds.count()) + std::to_string(nanoseconds.count());
    }

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

        

        //std::string image_filename = main_dir + "/" + camera_name + "/" + sec + "." + nsec + ".png";
        std::string t = get_timestamp() ;
        std::string image_filename = dir1 + t+ ".png";
        ordine_foto << t << std::endl;
        cv::imwrite(image_filename, cv_bridge_->image);
        RCLCPP_INFO(this->get_logger(), "Saved left image: %s", image_filename.c_str());

        image_count_1 ++;
    }

    void image2_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        //save_image(msg, "camera2", image_count_2);
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

    void save_image(const sensor_msgs::msg::Image::SharedPtr msg, const std::string &camera_name)
    {
        
        
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Write data to CSV file
        std::string t = get_timestamp();
        imu_csv_data << t << ","
                  << msg->angular_velocity.x << "," << msg->angular_velocity.y << ","
                  << msg->angular_velocity.z << "," << msg->linear_acceleration.x << ","
                  << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "\n";
        imu_csv_data.flush();


        int64_t sec = msg->header.stamp.sec;
        int64_t nsec = msg->header.stamp.nanosec;
        int64_t timestamp = sec * 1000000000 + nsec;


        imu_csv_matching_timestamp << timestamp << "," << t << std::endl;
        imu_csv_matching_timestamp.flush();

        RCLCPP_INFO(this->get_logger(), "Saved imu %s", t.c_str());

    }

    void handleSignal(int signal) {
        if (signal == SIGINT) {
            imu_csv_data.close();
            ordine_foto.close();
            imu_csv_matching_timestamp.close();
            rclcpp::shutdown();
        }
    }



    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::ofstream imu_csv_data, imu_csv_matching_timestamp;
    std::ofstream ordine_foto;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
    int image_count_1, image_count_2;
    // Create directories for saving images
    std::string main_dir = "mapping_bag_parcheggio_orario/";
    std::string dir1 = main_dir+"cam0/";
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
