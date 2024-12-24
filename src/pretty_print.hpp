#pragma once
#include <ostream>


namespace libcamera
{
class CameraManager;
class StreamFormats;
struct StreamConfiguration;
} // namespace libcamera

std::ostream &
operator<<(std::ostream &out, const libcamera::CameraManager &camera_manager);

std::ostream &
operator<<(std::ostream &out, const libcamera::StreamFormats &formats);

std::string
list_format_sizes(const libcamera::StreamConfiguration &configuration);
