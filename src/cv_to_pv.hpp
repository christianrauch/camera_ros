#pragma once
#include <rclcpp/parameter_value.hpp>
#include <stdexcept>
#include <string>

namespace libcamera
{
class ControlId;
class ControlValue;
} // namespace libcamera


class invalid_conversion : public std::runtime_error
{
public:
  explicit invalid_conversion(const std::string &msg) : std::runtime_error(msg) {}
};


rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value);

rclcpp::ParameterType
cv_to_pv_type(const libcamera::ControlId *const id);
