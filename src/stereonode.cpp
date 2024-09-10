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
            callback_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options;
            options.callback_group = callback_group2;
            stereo_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",  
            1,
            std::bind(&stereonode::image_callback, this,std::placeholders::_1),
            options);

            left_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", 1);
            right_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", 1);
            left_img_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", 1);
            right_img_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", 1);

            left_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), 
            std::bind(&stereonode::left_callback,this),
            callback_group3);
            
            right_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), 
            std::bind(&stereonode::right_callback,this),
            callback_group1);

            RCLCPP_INFO(this->get_logger(), "Stereo publisher initializing");
        }

    private:
    
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
        {   
           
            // (void)msg;HAHA never mind this, was using durring debugging
            RCLCPP_WARN(this->get_logger(),"SUBSCRIBER CALLBACK IS ACTIVE");
            //pushing image to the queues
            {    
                std::lock_guard<std::mutex> lock(queue_mutex1_);
                image_queue1_.push(msg);
                RCLCPP_WARN(this->get_logger(),"PUSHED TO QUEUE 1");
            }
            
            {
                std::lock_guard<std::mutex> lock(queue_mutex2_);
                image_queue2_.push(msg);
                RCLCPP_WARN(this->get_logger(),"PUSHED TO QUEUE 2");
            }
        }

        // void left_callback()
        // {
        //     RCLCPP_INFO(this->get_logger(), "THE LEFT TIMER IS ACTIVE");
        // }

        // void right_callback()
        // {
        //     RCLCPP_INFO(this->get_logger(), "THE RIGHT TIMER IS ACTIVE");

        // }
        
        void left_callback()
        {   
            RCLCPP_WARN(this->get_logger(),"started left_callback");
            sensor_msgs::msg::Image::SharedPtr msg;
            {
                std::lock_guard<std::mutex> lock(queue_mutex1_);
                if (!image_queue1_.empty())
                {
                    msg = image_queue1_.front();
                    image_queue1_.pop();
                }
            }
            
            if (msg)
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

                // Splitting off the left half of the image
                cv::Mat left_image = cv_image(cv::Rect(0, 0, 1280, 720));   

                // Convert the left image back to ROS Image message
                cv_bridge::CvImage left_cv_image;
                left_cv_image.header = msg->header; // Use the same timestamp and frame ID
                left_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
                left_cv_image.image = left_image;
                auto left_msg = left_cv_image.toImageMsg(); // Convert to ROS Image message

                // Publish the left image
                left_img_pub->publish(*left_msg);
                RCLCPP_WARN(this->get_logger(), "Left Image has been published");

                // Create and publish CameraInfo messages for left image
                auto left_camera_info = create_camera_info(msg->header, 1280, 720);

                // Publish the camera info
                left_img_info->publish(left_camera_info);
            }
            else
            {
                // No image to process, sleep for a short time
                RCLCPP_WARN(this->get_logger(),"NO IMAGE");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }    

        }

        void right_callback()
        {   
            RCLCPP_WARN(this->get_logger(),"Started right_callback");
            sensor_msgs::msg::Image::SharedPtr msg;
            {
                std::lock_guard<std::mutex> lock(queue_mutex2_);
                if (!image_queue2_.empty())
                {
                    msg = image_queue2_.front();
                    image_queue2_.pop();
                }
            }

            if (msg)
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

                // Splitting off the right half of the image
                cv::Mat right_image = cv_image(cv::Rect(1280, 0, 1280, 720)); 

                // Convert the right image back to ROS Image message
                cv_bridge::CvImage right_cv_image;
                right_cv_image.header = msg->header; // Use the same timestamp and frame ID
                right_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
                right_cv_image.image = right_image;
                auto right_msg = right_cv_image.toImageMsg(); // Convert to ROS Image message

                // Publish the right image
                right_img_pub->publish(*right_msg);
                RCLCPP_WARN(this->get_logger(), "Right Image has been published");

                // Create and publish CameraInfo messages forright image
                /*COMMENTED OUT THIS PART TO TEST FIRST*/
                auto right_camera_info = create_camera_info(msg->header, 1280, 720);

                // Publish the camera info
                right_img_info->publish(right_camera_info);
            }
            else
            {
                // No image to process, sleep for a short time
                RCLCPP_WARN(this->get_logger(),"NO IMAGE");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

        }

        sensor_msgs::msg::CameraInfo create_camera_info(const std_msgs::msg::Header& header, int width, int height)
        {
            sensor_msgs::msg::CameraInfo camera_info;
            camera_info.header = header;
            camera_info.width = width;
            camera_info.height = height;

            // Assuming arbitrary calibration data for testing
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

            // Load actual calibration data (K, P matrices, etc.) for actual measurements
            return camera_info;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_subscription;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_img_info;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_img_info;
        std::queue<sensor_msgs::msg::Image::SharedPtr> image_queue1_;
        std::queue<sensor_msgs::msg::Image::SharedPtr> image_queue2_;
        std::mutex queue_mutex1_;
        std::mutex queue_mutex2_;
        rclcpp::TimerBase::SharedPtr left_timer_;
        rclcpp::TimerBase::SharedPtr right_timer_;
        rclcpp::CallbackGroup::SharedPtr callback_group1;
        rclcpp::CallbackGroup::SharedPtr callback_group2;
        rclcpp::CallbackGroup::SharedPtr callback_group3;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereonode>(); 
    rclcpp::executors::MultiThreadedExecutor executor_(rclcpp::ExecutorOptions(), 4);  
    executor_.add_node(node);
    executor_.spin();
    rclcpp::shutdown();
    return 0;
}