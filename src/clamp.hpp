#pragma once
#include "control_type_map.hpp"
#include <libcamera/controls.h>
#include <rclcpp/parameter_value.hpp>


namespace std
{

CTRectangle clamp(const CTRectangle &val, const CTRectangle &lo, const CTRectangle &hi)
{
  const int x = std::clamp(val.x, lo.x, hi.x);
  const int y = std::clamp(val.y, lo.y, hi.y);
  unsigned int width = std::clamp(x + val.width, lo.x + lo.width, hi.x + hi.width) - x;
  unsigned int height = std::clamp(y + val.height, lo.y + lo.height, hi.y + hi.height) - y;

  return CTRectangle {x, y, width, height};
}

} // namespace std


template<typename T>
libcamera::ControlValue clamp_array(const libcamera::ControlValue &value,
                                    const libcamera::ControlValue &min,
                                    const libcamera::ControlValue &max)
{
  const libcamera::Span<const T> v = value.get<libcamera::Span<const T>>();
  const libcamera::Span<const T> a = min.get<libcamera::Span<const T>>();
  const libcamera::Span<const T> b = max.get<libcamera::Span<const T>>();

  std::vector<T> vclamp(v.size());

  for (size_t i = 0; i < v.size(); i++)
    vclamp[i] = std::clamp(v[i], a[i], b[i]);

  return libcamera::ControlValue(libcamera::Span<const T>(vclamp));
}

template<typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<T>, CTBool>::value, bool> = true>
libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue &min,
                              const libcamera::ControlValue &max)
{
  if (value.isArray()) {
    return clamp_array<T>(value, min, max);
  }
  else {
    return std::clamp(value.get<T>(), min.get<T>(), max.get<T>());
    //    return clamp_scalar<T>(value, min, max);
  }
}

template<typename T,
         std::enable_if_t<std::is_same<std::remove_cv_t<T>, CTBool>::value, bool> = true>
libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue & /*min*/,
                              const libcamera::ControlValue & /*max*/)
{
  return value;
}

//#define CASE_CLAMP(T)                                                                              \
//  case libcamera::ControlType##T:                                                                  \
//    return clamp<ControlTypeMap<libcamera::ControlType##T>::type>(value, min, max);

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

template<typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
rclcpp::ParameterValue control_array_to_pv(const std::vector<T> &values)
{
  return rclcpp::ParameterValue(values);
}

template<typename T, std::enable_if_t<!std::is_arithmetic<T>::value, bool> = true>
rclcpp::ParameterValue control_array_to_pv(const std::vector<T> & /*values*/)
{
  throw std::runtime_error("ParameterValue only supported for arithmetic types");
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

  return {};
}

template<typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
T min(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    const libcamera::Span<const T> v = value.get<libcamera::Span<const T>>();
    return *std::min_element(v.begin(), v.end());
  }
  else {
    return value.get<T>();
  }
}

template<typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
T max(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    const libcamera::Span<const T> v = value.get<libcamera::Span<const T>>();
    return *std::max_element(v.begin(), v.end());
  }
  else {
    return value.get<T>();
  }
}
