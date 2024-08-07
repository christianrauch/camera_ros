#include "type_extent.hpp"
#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/version.h>
#include <stdexcept>
#include <string>
#include <type_traits>


#define LIBCAMERA_VER_GE(major, minor, patch)                               \
  ((major < LIBCAMERA_VERSION_MAJOR) ||                                     \
   (major == LIBCAMERA_VERSION_MAJOR && minor < LIBCAMERA_VERSION_MINOR) || \
   (major == LIBCAMERA_VERSION_MAJOR && minor == LIBCAMERA_VERSION_MINOR && \
    patch <= LIBCAMERA_VERSION_PATCH))


template<typename T, std::enable_if_t<!libcamera::details::is_span<T>::value, bool> = true>
std::size_t
get_extent(const libcamera::Control<T> &)
{
  return 0;
}

template<typename T, std::enable_if_t<libcamera::details::is_span<T>::value, bool> = true>
std::size_t
get_extent(const libcamera::Control<T> &)
{
  return libcamera::Control<T>::type::extent;
}

#define IF(T)                                  \
  if (id->id() == libcamera::controls::T.id()) \
    return get_extent(libcamera::controls::T);


std::size_t
get_extent(const libcamera::ControlId *id)
{
#if LIBCAMERA_VER_GE(0, 1, 0)
  IF(AeEnable)
  IF(AeLocked)
  IF(AeMeteringMode)
  IF(AeConstraintMode)
  IF(AeExposureMode)
  IF(ExposureValue)
  IF(ExposureTime)
  IF(AnalogueGain)
  IF(Brightness)
  IF(Contrast)
  IF(Lux)
  IF(AwbEnable)
  IF(AwbMode)
  IF(AwbLocked)
  IF(ColourGains)
  IF(ColourTemperature)
  IF(Saturation)
  IF(SensorBlackLevels)
  IF(Sharpness)
  IF(FocusFoM)
  IF(ColourCorrectionMatrix)
  IF(ScalerCrop)
  IF(DigitalGain)
  IF(FrameDuration)
  IF(FrameDurationLimits)
  IF(SensorTimestamp)
  IF(AfMode)
  IF(AfRange)
  IF(AfSpeed)
  IF(AfMetering)
  IF(AfWindows)
  IF(AfTrigger)
  IF(AfPause)
  IF(LensPosition)
  IF(AfState)
  IF(AfPauseState)
#endif

#if LIBCAMERA_VER_GE(0, 2, 0)
  IF(HdrMode)
  IF(AeFlickerPeriod)
  IF(AeFlickerMode)
#ifdef LIBCAMERA_HAS_RPI_VENDOR_CONTROLS
  IF(rpi::StatsOutputEnable)
  IF(rpi::Bcm2835StatsOutput)
#endif
#endif

  throw std::runtime_error("control " + id->name() + " (" + std::to_string(id->id()) +
                           ") not handled");
}
