#pragma once
#include <libcamera/controls.h>


libcamera::ControlValue
cast_cv(const libcamera::ControlValue &value, const libcamera::ControlType target_type);
