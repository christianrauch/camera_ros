#pragma once
//#include "sensor_msgs/msg/detail/compressed_image__struct.hpp"
//#include <cstdint>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

sensor_msgs::msg::Image::UniquePtr
//decompress(const std::vector<uint8_t> &data);
//decompress(const sensor_msgs::msg::CompressedImage &data);
decompress(const sensor_msgs::msg::CompressedImage::UniquePtr &data);
//decompress(const sensor_msgs::msg::CompressedImage::SharedPtr &data);
