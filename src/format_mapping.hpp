#pragma once
#include <string>

namespace libcamera
{
class PixelFormat;
class StreamFormats;
} // namespace libcamera

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

libcamera::StreamFormats
get_common_stream_formats(const libcamera::StreamFormats &formats);
