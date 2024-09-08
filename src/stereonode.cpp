#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

class stereonode : public rclcpp::Node 
{
public:
    stereonode() : Node("stereonode") 
    {
        stereo_subscription = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&stereonode::stereo_callback, this, std::placeholders::_1));
        left_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", 10);
        right_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", 10);
        left_img_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", 10);
        right_img_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", 10);

        RCLCPP_INFO(this->get_logger(), "Stereo publisher initializing");
    }

private:
    void stereo_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Converting ROS Image message to cv format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Getting the OpenCV image from the above operation
        cv::Mat cv_image = cv_ptr->image;

        // Checking if the image size is 2560x720
        if (cv_image.cols != 2560 || cv_image.rows != 720)
        {
            RCLCPP_ERROR(this->get_logger(), "Unexpected image size: %d x %d", cv_image.cols, cv_image.rows);
            return;
        }

        // Splitting the image into left and right halves
        cv::Mat left_image = cv_image(cv::Rect(0, 0, 1280, 720));   
        cv::Mat right_image = cv_image(cv::Rect(1280, 0, 1280, 720)); 

        // Convert the left image back to ROS Image message
        cv_bridge::CvImage left_cv_image;
        left_cv_image.header = msg->header; // Use the same timestamp and frame ID
        left_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        left_cv_image.image = left_image;
        auto left_msg = left_cv_image.toImageMsg(); // Convert to ROS Image message

        // Convert the right image back to ROS Image message
        cv_bridge::CvImage right_cv_image;
        right_cv_image.header = msg->header; // Use the same timestamp and frame ID
        right_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        right_cv_image.image = right_image;
        auto right_msg = right_cv_image.toImageMsg(); // Convert to ROS Image message

        // Publish the left and right images
        left_img_pub->publish(*left_msg);
        right_img_pub->publish(*right_msg);

        // Create and publish CameraInfo messages for left and right cameras
        auto left_camera_info = create_camera_info(msg->header, 1280, 720);
        auto right_camera_info = create_camera_info(msg->header, 1280, 720);

        // Publish the camera info
        left_img_info->publish(left_camera_info);
        right_img_info->publish(right_camera_info);
    }

    sensor_msgs::msg::CameraInfo create_camera_info(const std_msgs::msg::Header& header, int width, int height)
    {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header = header;
        camera_info.width = width;
        camera_info.height = height;

        // Assuming a pinhole camera model; fill with dummy calibration data for testing
        camera_info.k = {
            static_cast<double>(width), 0.0, static_cast<double>(width) / 2.0,
            0.0, static_cast<double>(height), static_cast<double>(height) / 2.0,
            0.0, 0.0, 1.0
        };  // Intrinsic matrix

        camera_info.p = {
            static_cast<double>(width), 0.0, static_cast<double>(width) / 2.0, 0.0,
            0.0, static_cast<double>(height), static_cast<double>(height) / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        };  // Projection matrix

        // For real applications, you should load actual calibration data (K, P matrices, etc.)
        return camera_info;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_subscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_img_info;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_img_info;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereonode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}