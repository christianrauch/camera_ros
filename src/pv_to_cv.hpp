#pragma once
#include <libcamera/controls.h>


namespace rclcpp
{
class Parameter;
}

libcamera::ControlValue
pv_to_cv(const rclcpp::Parameter &parameter, const libcamera::ControlType &type);
