#include "format_mapping.hpp"
#include "libcamera_version_utils.hpp"
#include <cstdint>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/stream.h>
#include <map>
#include <sensor_msgs/image_encodings.hpp>
#include <unordered_map>
#include <utility>
#include <vector>


namespace cam = libcamera::formats;
namespace ros = sensor_msgs::image_encodings;

// mapping of FourCC to ROS image encodings
// references:
// - full list of FourCC: 'include/uapi/drm/drm_fourcc.h'
// - V4L2 image formats: 'https://docs.kernel.org/userspace-api/media/v4l/pixfmt.html'

// supported FourCC formats, without conversion
static const std::unordered_map<uint32_t, std::string> map_format_raw = {
  // RGB encodings
  // NOTE: Following the DRM definition, RGB formats codes are stored in little-endian order.
  {cam::R8.fourcc(), ros::MONO8},
#if LIBCAMERA_VER_GE(0, 3, 0)
  {cam::R16.fourcc(), ros::MONO16},
#endif
  {cam::RGB888.fourcc(), ros::BGR8},
  {cam::BGR888.fourcc(), ros::RGB8},
#if LIBCAMERA_VER_GE(0, 3, 0)
  {cam::RGB161616.fourcc(), ros::BGR16},
  {cam::BGR161616.fourcc(), ros::RGB16},
#endif
  {cam::XRGB8888.fourcc(), ros::BGRA8},
  {cam::XBGR8888.fourcc(), ros::RGBA8},
  {cam::ARGB8888.fourcc(), ros::BGRA8},
  {cam::ABGR8888.fourcc(), ros::RGBA8},
  // YUV encodings
  {cam::YUYV.fourcc(), ros::YUV422_YUY2},
  {cam::UYVY.fourcc(), ros::YUV422},
  {cam::NV21.fourcc(), ros::NV21},
  {cam::NV24.fourcc(), ros::NV24},
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
static const std::unordered_map<uint32_t, std::string> map_format_compressed = {
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

libcamera::StreamFormats
get_common_stream_formats(const libcamera::StreamFormats &formats)
{
  std::map<libcamera::PixelFormat, std::vector<libcamera::SizeRange>> common_stream_formats;
  for (const libcamera::PixelFormat &fmt : formats.pixelformats()) {
    if (format_type(fmt) != FormatType::NONE) {
      common_stream_formats[fmt] = {formats.range(fmt)};
    }
  }

  return {common_stream_formats};
}
