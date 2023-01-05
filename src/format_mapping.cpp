#include "format_mapping.hpp"
#include <cstdint>
#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>
#include <sensor_msgs/image_encodings.hpp>
#include <unordered_map>


namespace cam = libcamera::formats;
namespace ros = sensor_msgs::image_encodings;

//mapping of FourCC to ROS image encodings
// see 'include/uapi/drm/drm_fourcc.h' for a full FourCC list

// supported FourCC formats, without conversion
const std::unordered_map<uint32_t, std::string> map_format_raw = {
  // RGB encodings
  // NOTE: Following the DRM definition, RGB formats codes are stored in little-endian order.
  {cam::R8.fourcc(), ros::MONO8},
  {cam::RGB888.fourcc(), ros::BGR8},
  {cam::BGR888.fourcc(), ros::RGB8},
  {cam::XRGB8888.fourcc(), ros::BGRA8},
  {cam::XBGR8888.fourcc(), ros::RGBA8},
  {cam::ARGB8888.fourcc(), ros::BGRA8},
  {cam::ABGR8888.fourcc(), ros::RGBA8},
  // YUV encodings
  {cam::YUYV.fourcc(), ros::YUV422_YUY2},
  {cam::UYVY.fourcc(), ros::YUV422},
  // Bayer encodings
  {cam::SRGGB8.fourcc(), ros::BAYER_RGGB8},
  {cam::SGRBG8.fourcc(), ros::BAYER_GRBG8},
  {cam::SGBRG8.fourcc(), ros::BAYER_GBRG8},
  {cam::SBGGR8.fourcc(), ros::BAYER_BGGR8},
  {cam::SRGGB16.fourcc(), ros::BAYER_RGGB16},
  {cam::SGRBG16.fourcc(), ros::BAYER_GRBG16},
  {cam::SGBRG16.fourcc(), ros::BAYER_GBRG16},
  {cam::SBGGR16.fourcc(), ros::BAYER_BGGR16},
};

// supported FourCC formats, without conversion, compressed
const std::unordered_map<uint32_t, std::string> map_format_compressed = {
  {cam::MJPEG.fourcc(), "jpeg"},
};

std::string
get_ros_encoding(const libcamera::PixelFormat &pixelformat)
{
  if (map_format_raw.count(pixelformat.fourcc()))
    return map_format_raw.at(pixelformat.fourcc());
  if (map_format_compressed.count(pixelformat.fourcc()))
    return map_format_compressed.at(pixelformat.fourcc());

  return {};
}

FormatType
format_type(const libcamera::PixelFormat &pixelformat)
{
  if (map_format_raw.count(pixelformat.fourcc()))
    return FormatType::RAW;
  if (map_format_compressed.count(pixelformat.fourcc()))
    return FormatType::COMPRESSED;
  return FormatType::NONE;
}
