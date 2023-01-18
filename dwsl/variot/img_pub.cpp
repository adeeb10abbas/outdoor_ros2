// Author: Adeeb Abbas
// Date: 1/14/2022
// Description: This is a ROS2 node that publishes images to the variot/images topic
// This is a test node to test the variot camera and the variot/images topic
// This will be expanded to publish many different types of data
// LICENSE: MIT

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

class variotImage : public rclcpp::Node
{
  public:
    variotImage()
    : Node("variotImage_pub"), count_(0)
    {
      // Take in the image from the variot camera
      // Publish the image to the variot/images topic
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("variot/images", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&variotImage::img_callback, this));
    }

  private:
    void img_callback()
    {
      // Main Callback 
      /*
      Takes in the image and publishes it to the variot/images topic
      This will be expanded to many different types of data for now it's only images
      */
      auto message = sensor_msgs::msg::Image();
      // Create a random image of size 640x480 to publish
      message.header.stamp = this->now(); // time stamp
      message.header.frame_id = "variot"; // frame name
      message.height = 480; 
      message.width = 640; 
      message.encoding = "bgr8"; // 8 bits per channel
      message.is_bigendian = 0; // 
      message.step = 640 * 3; // 3 bytes per pixel
      message.data = std::vector<uint8_t>(640 * 480 * 3, 0); // random data to fill the image of size 640x480
      // Publish the image!
      publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Publishing Image with Data: '%s'", message.header.frame_id.c_str());
      //Increment the counter
      count_++;
      // Publish a sample file to the variot/images topic
      // Check if the file exists
      std::string path = "dwsl/variot/sample_data/sample_img.png";
      if (!rcpputils::fs::exists(path)) {
        RCLCPP_ERROR(this->get_logger(), "File does not exist");
        return;
      }
      cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
      cv_bridge::CvImage cv_image;
      cv_image.image = image;
      cv_image.encoding = "bgr8";
      cv_image.header.stamp = this->now();
      cv_image.header.frame_id = "variot";
      publisher_->publish(*cv_image.toImageMsg());
      RCLCPP_INFO(this->get_logger(), "Publishing Image with Data: '%s'", cv_image.header.frame_id.c_str());

      //[TODO:adeeb10abbas] Add publishing to websockets

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Main loop
  rclcpp::spin(std::make_shared<variotImage>());
  rclcpp::shutdown();
  return 0;
}