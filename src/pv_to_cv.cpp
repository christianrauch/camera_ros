#include "pv_to_cv.hpp"
#include "types.hpp"
#include <cstdint>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <vector>


libcamera::ControlValue
pv_to_cv_int_array(const std::vector<int64_t> &values, const libcamera::ControlType &type)
{
  // convert to Span (Integer32, Integer64) or geometric type Rectangle, Size
  switch (type) {
  case libcamera::ControlTypeInteger32:
    return {
      libcamera::Span<const CTInteger32>(std::vector<CTInteger32>(values.begin(), values.end()))};
  case libcamera::ControlTypeInteger64:
    return {libcamera::Span<const CTInteger64>(values)};
  case libcamera::ControlTypeRectangle:
    return {libcamera::Rectangle(values[0], values[1], values[2], values[3])};
  case libcamera::ControlTypeSize:
    return {libcamera::Size(values[0], values[1])};
  default:
    return {};
  }
}

libcamera::ControlValue
pv_to_cv(const rclcpp::Parameter &parameter, const libcamera::ControlType &type)
{
  switch (parameter.get_type()) {
  case rclcpp::ParameterType::PARAMETER_NOT_SET:
    return {};
  case rclcpp::ParameterType::PARAMETER_BOOL:
    return {parameter.as_bool()};
  case rclcpp::ParameterType::PARAMETER_INTEGER:
    if (type == libcamera::ControlTypeInteger32)
      return {CTInteger32(parameter.as_int())};
    else if (type == libcamera::ControlTypeInteger64)
      return {CTInteger64(parameter.as_int())};
    else
      return {};
  case rclcpp::ParameterType::PARAMETER_DOUBLE:
    return {CTFloat(parameter.as_double())};
  case rclcpp::ParameterType::PARAMETER_STRING:
    return {parameter.as_string()};
  case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    return {libcamera::Span<const CTByte>(parameter.as_byte_array())};
  case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    return {};
  case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    return pv_to_cv_int_array(parameter.as_integer_array(), type);
  case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
  {
    // convert to float vector
    return {libcamera::Span<const CTFloat>(
      std::vector<CTFloat>(parameter.as_double_array().begin(), parameter.as_double_array().end()))};
  }
  case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    return {libcamera::Span<const CTString>(parameter.as_string_array())};
  default:
    return {};
  }
}
