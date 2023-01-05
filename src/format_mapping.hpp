#pragma once
#include <string>

namespace libcamera
{
class PixelFormat;
}

enum class FormatType
{
  NONE,
  RAW,
  COMPRESSED,
};

std::string
get_ros_encoding(const libcamera::PixelFormat &pixelformat);

FormatType
format_type(const libcamera::PixelFormat &pixelformat);
