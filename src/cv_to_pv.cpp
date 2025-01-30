#include "cv_to_pv.hpp"
#include "libcamera_version_utils.hpp"
#include "type_extent.hpp"
#include "types.hpp"
#include <cstdint>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <rclcpp/parameter_value.hpp>
#include <string>
#include <type_traits>
#include <vector>


#define CASE_CONVERT(T)           \
  case libcamera::ControlType##T: \
    return cv_to_pv(extract_value<ControlTypeMap<libcamera::ControlType##T>::type>(value));

#define CASE_NONE(T)              \
  case libcamera::ControlType##T: \
    return {};


template<typename T>
std::vector<T>
extract_value(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    const libcamera::Span<const T> span = value.get<libcamera::Span<const T>>();
    return std::vector<T>(span.begin(), span.end());
  }
  else {
    return {value.get<T>()};
  }
}

template<
  typename T,
  std::enable_if_t<std::is_constructible<rclcpp::ParameterValue, std::vector<T>>::value, bool> = true>
rclcpp::ParameterValue
cv_to_pv_array(const std::vector<T> &values)
{
  return rclcpp::ParameterValue(values);
}

template<
  typename T,
  std::enable_if_t<!std::is_constructible<rclcpp::ParameterValue, std::vector<T>>::value, bool> = true>
rclcpp::ParameterValue
cv_to_pv_array(const std::vector<T> & /*values*/)
{
  throw invalid_conversion("ParameterValue not constructible from complex type.");
}

template<
  typename T,
  std::enable_if_t<std::is_constructible<rclcpp::ParameterValue, T>::value, bool> = true>
rclcpp::ParameterValue
cv_to_pv_scalar(const T &value)
{
  return rclcpp::ParameterValue(value);
}

rclcpp::ParameterValue
cv_to_pv_scalar(const uint16_t &val)
{
  return rclcpp::ParameterValue(static_cast<int32_t>(val));
}

rclcpp::ParameterValue
cv_to_pv_scalar(const uint32_t &val)
{
  return rclcpp::ParameterValue(static_cast<int64_t>(val));
}

rclcpp::ParameterValue
cv_to_pv_scalar(const libcamera::Rectangle &rectangle)
{
  return rclcpp::ParameterValue(
    std::vector<int64_t> {rectangle.x, rectangle.y, rectangle.width, rectangle.height});
}

rclcpp::ParameterValue
cv_to_pv_scalar(const libcamera::Size &size)
{
  return rclcpp::ParameterValue(std::vector<int64_t> {size.width, size.height});
}

#if LIBCAMERA_VER_GE(0, 4, 0)
rclcpp::ParameterValue
cv_to_pv_scalar(const libcamera::Point &point)
{
  return rclcpp::ParameterValue(std::vector<int64_t> {point.x, point.y});
}
#endif

template<typename T>
rclcpp::ParameterValue
cv_to_pv(const std::vector<T> &values)
{
  switch (values.size()) {
  case 0:
    // empty array
    return rclcpp::ParameterValue();
  case 1:
    // single element (scalar)
    return cv_to_pv_scalar(values[0]);
  default:
    // dynamic array
    return cv_to_pv_array(values);
  }
}

rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value)
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
#if LIBCAMERA_VER_GE(0, 4, 0)
    CASE_CONVERT(Unsigned16)
    CASE_CONVERT(Unsigned32)
    CASE_CONVERT(Point)
#endif
  }

  return {};
}

rclcpp::ParameterType
cv_to_pv_type(const libcamera::ControlId *const id)
{
  if (get_extent(id) == 0) {
    switch (id->type()) {
    case libcamera::ControlType::ControlTypeNone:
      throw unsupported_control(id);
    case libcamera::ControlType::ControlTypeBool:
      return rclcpp::ParameterType::PARAMETER_BOOL;
    case libcamera::ControlType::ControlTypeByte:
#if LIBCAMERA_VER_GE(0, 4, 0)
    case libcamera::ControlType::ControlTypeUnsigned16:
    case libcamera::ControlType::ControlTypeUnsigned32:
#endif
    case libcamera::ControlType::ControlTypeInteger32:
    case libcamera::ControlType::ControlTypeInteger64:
      return rclcpp::ParameterType::PARAMETER_INTEGER;
    case libcamera::ControlType::ControlTypeFloat:
      return rclcpp::ParameterType::PARAMETER_DOUBLE;
    case libcamera::ControlType::ControlTypeString:
      return rclcpp::ParameterType::PARAMETER_STRING;
    case libcamera::ControlType::ControlTypeRectangle:
      return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    case libcamera::ControlType::ControlTypeSize:
      return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
#if LIBCAMERA_VER_GE(0, 4, 0)
    case libcamera::ControlType::ControlTypePoint:
      return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
#endif
    }
  }
  else {
    switch (id->type()) {
    case libcamera::ControlType::ControlTypeNone:
      throw unsupported_control(id);
    case libcamera::ControlType::ControlTypeBool:
      return rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
    case libcamera::ControlType::ControlTypeByte:
#if LIBCAMERA_VER_GE(0, 4, 0)
    case libcamera::ControlType::ControlTypeUnsigned16:
    case libcamera::ControlType::ControlTypeUnsigned32:
#endif
    case libcamera::ControlType::ControlTypeInteger32:
    case libcamera::ControlType::ControlTypeInteger64:
      return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    case libcamera::ControlType::ControlTypeFloat:
      return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
    case libcamera::ControlType::ControlTypeString:
      return rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    case libcamera::ControlType::ControlTypeRectangle:
      throw unsupported_control(id);
    case libcamera::ControlType::ControlTypeSize:
      throw unsupported_control(id);
#if LIBCAMERA_VER_GE(0, 4, 0)
    case libcamera::ControlType::ControlTypePoint:
      throw unsupported_control(id);
#endif
    }
  }

  throw should_not_reach();
}
