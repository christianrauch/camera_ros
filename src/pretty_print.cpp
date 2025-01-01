#include "pretty_print.hpp"
#include <cstddef>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>


std::ostream &
operator<<(std::ostream &out, const libcamera::CameraManager &camera_manager)
{
  out << std::endl
      << ">> cameras:";
  for (size_t id = 0; id < camera_manager.cameras().size(); id++) {
    const std::shared_ptr<libcamera::Camera> camera = camera_manager.cameras().at(id);
    const std::string name =
      camera->properties().get(libcamera::properties::Model).value_or("UNDEFINED");
    out << std::endl
        << "   " << id << ": " << name << " (" << camera->id() << ")";
  }
  return out;
}

std::ostream &
operator<<(std::ostream &out, const libcamera::StreamFormats &formats)
{
  // show supported pixel formats
  out << std::endl
      << ">> stream formats:";
  for (const libcamera::PixelFormat &pixelformat : formats.pixelformats()) {
    out << std::endl
        << "   - " << pixelformat.toString() << " ("
        << formats.range(pixelformat).min.toString() << " - "
        << formats.range(pixelformat).max.toString() << ")";
  }
  return out;
}

std::string
list_format_sizes(const libcamera::StreamConfiguration &configuration)
{
  std::ostringstream out;
  out << std::endl
      << ">> " << configuration.pixelFormat << " format sizes:";
  for (const libcamera::Size &size : configuration.formats().sizes(configuration.pixelFormat))
    out << std::endl
        << "   - " << size.toString();
  return out.str();
}
