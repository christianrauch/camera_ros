#include "type_extent.hpp"


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

#define IF(T)                                                                                      \
  if (id->id() == libcamera::controls::T.id())                                                     \
    return get_extent(libcamera::controls::T);


std::size_t
get_extent(const libcamera::ControlId *id)
{
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

  throw std::runtime_error("control " + id->name() + " (" + std::to_string(id->id()) +
                           ") not handled");
}
