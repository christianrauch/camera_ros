#pragma once
#include "control_type_map.hpp"
#include <libcamera/controls.h>
#include <rclcpp/parameter_value.hpp>


template<typename T>
libcamera::ControlValue clamp_scalar(const libcamera::ControlValue &value,
                                     const libcamera::ControlValue &min,
                                     const libcamera::ControlValue &max)
{
  //  std::cout << value.isArray() << " / scalar" << std::endl;
  return std::clamp(value.get<T>(), min.get<T>(), max.get<T>());
}

template<>
libcamera::ControlValue clamp_scalar<libcamera::Rectangle>(const libcamera::ControlValue &value,
                                                           const libcamera::ControlValue &min,
                                                           const libcamera::ControlValue &max)
{
  const libcamera::Rectangle &rv = value.get<libcamera::Rectangle>();
  const libcamera::Rectangle &rmin = min.get<libcamera::Rectangle>();
  const libcamera::Rectangle &rmax = max.get<libcamera::Rectangle>();

  const int x = std::clamp(rv.x, rmin.x, rmax.x);
  const int y = std::clamp(rv.y, rmin.y, rmax.y);
  unsigned int width = std::clamp(x + rv.width, rmin.x + rmin.width, rmax.x + rmax.width) - x;
  unsigned int height = std::clamp(y + rv.height, rmin.y + rmin.height, rmax.y + rmax.height) - y;

  return libcamera::Rectangle {x, y, width, height};
}

template<typename T>
libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue &min,
                              const libcamera::ControlValue &max)
{
  if (value.isArray()) {
    //    return clamp_scalar<T>(value, min, max);
    return {};
  }
  else {
    return clamp_scalar<T>(value, min, max);
  }
}

libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue &min,
                              const libcamera::ControlValue &max)
{
  if (min.type() != max.type())
    throw std::runtime_error("minimum and maximum types mismatch");

  switch (value.type()) {
  case libcamera::ControlTypeNone:
    return {};
  case libcamera::ControlTypeBool:
    return clamp<ControlTypeMap<libcamera::ControlTypeBool>::type>(value, min, max);
  case libcamera::ControlTypeByte:
    return clamp<ControlTypeMap<libcamera::ControlTypeByte>::type>(value, min, max);
  case libcamera::ControlTypeInteger32:
    return clamp<ControlTypeMap<libcamera::ControlTypeInteger32>::type>(value, min, max);
  case libcamera::ControlTypeInteger64:
    return clamp<ControlTypeMap<libcamera::ControlTypeInteger64>::type>(value, min, max);
  case libcamera::ControlTypeFloat:
    return clamp<ControlTypeMap<libcamera::ControlTypeFloat>::type>(value, min, max);
  case libcamera::ControlTypeString:
    return clamp<ControlTypeMap<libcamera::ControlTypeString>::type>(value, min, max);
  case libcamera::ControlTypeRectangle:
    return clamp<ControlTypeMap<libcamera::ControlTypeRectangle>::type>(value, min, max);
  case libcamera::ControlTypeSize:
    return clamp<ControlTypeMap<libcamera::ControlTypeSize>::type>(value, min, max);
  }

  return {};
}

template<typename T>
std::vector<T> extract_value(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    const libcamera::Span<const T> span = value.get<libcamera::Span<const T>>();
    return std::vector<T>(span.begin(), span.end());
  }
  else {
    return {value.get<T>()};
  }
}

template<typename T, std::enable_if_t<std::is_integral<T>::value, bool> = true>
rclcpp::ParameterValue control_array_to_pv(const std::vector<T> &values)
{
  return rclcpp::ParameterValue(values);
}

template<typename T, std::enable_if_t<!std::is_integral<T>::value, bool> = true>
rclcpp::ParameterValue control_array_to_pv(const std::vector<T> & /*values*/)
{
  throw std::runtime_error("ParameterValue only supported for integral types");
}

template<typename T>
rclcpp::ParameterValue control_to_pv(const std::vector<T> &values)
{
  if (values.size() > 1)
    return control_array_to_pv(values);
  else if (values.size() == 1)
    return rclcpp::ParameterValue(values[0]);
  else
    return rclcpp::ParameterValue();
}

#define CASE_CONVERT(T)                                                                            \
  case libcamera::ControlType##T:                                                                  \
    return control_to_pv(extract_value<ControlTypeMap<libcamera::ControlType##T>::type>(value));

#define CASE_NONE(T)                                                                               \
  case libcamera::ControlType##T:                                                                  \
    return {};

rclcpp::ParameterValue control_to_pv(const libcamera::ControlValue &value)
{
  switch (value.type()) {
    CASE_NONE(None)
    CASE_CONVERT(Bool)
    CASE_CONVERT(Byte)
    CASE_CONVERT(Integer32)
    CASE_CONVERT(Integer64)
    CASE_CONVERT(Float)
    CASE_CONVERT(String)
    CASE_CONVERT(Rectangle)
    CASE_CONVERT(Size)
  }
}
