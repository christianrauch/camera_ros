#pragma once
#include "exceptions.hpp"
#include <rclcpp/parameter_value.hpp>
#include <string>

namespace libcamera
{
class ControlId;
class ControlValue;
} // namespace libcamera


rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value);

rclcpp::ParameterType
cv_to_pv_type(const libcamera::ControlId *const id);
