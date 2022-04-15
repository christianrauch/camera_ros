#include "cv_to_pv.hpp"
#include "cast_cv.hpp"
#include "types.hpp"
#include <rclcpp/parameter_value.hpp>


#define CASE_CONVERT(T)                                                                            \
  case libcamera::ControlType##T:                                                                  \
    return cv_to_pv(extract_value<ControlTypeMap<libcamera::ControlType##T>::type>(value), extent);

#define CASE_NONE(T)                                                                               \
  case libcamera::ControlType##T:                                                                  \
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

template<typename T,
         std::enable_if_t<std::is_arithmetic<T>::value || std::is_same<std::string, T>::value,
                          bool> = true>
rclcpp::ParameterValue
cv_to_pv_array(const std::vector<T> &values)
{
  return rclcpp::ParameterValue(values);
}

template<typename T,
         std::enable_if_t<!std::is_arithmetic<T>::value && !std::is_same<std::string, T>::value,
                          bool> = true>
rclcpp::ParameterValue
cv_to_pv_array(const std::vector<T> & /*values*/)
{
  throw std::runtime_error("ParameterValue only supported for arithmetic types");
}

template<typename T,
         std::enable_if_t<std::is_arithmetic<T>::value || std::is_same<std::string, T>::value,
                          bool> = true>
rclcpp::ParameterValue
cv_to_pv_scalar(const T &value)
{
  return rclcpp::ParameterValue(value);
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

template<typename T>
rclcpp::ParameterValue
cv_to_pv(const std::vector<T> &values, const std::size_t &extent)
{
  if ((values.size() > 1 && extent > 1) && (values.size() != extent))
    throw std::runtime_error("type extent (" + std::to_string(extent) + ") and value size (" +
                             std::to_string(values.size()) +
                             ") cannot be larger than 1 and differ");

  if (values.size() > 1)
    return cv_to_pv_array(values);
  else if (values.size() == 1)
    if (!extent)
      return cv_to_pv_scalar(values[0]);
    else
      return cv_to_pv_array(std::vector<T>(extent, values[0]));
  else
    return rclcpp::ParameterValue();
}

rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value, const std::size_t &extent)
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
