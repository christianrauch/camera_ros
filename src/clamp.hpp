#pragma once
#include "types.hpp"
#include <libcamera/controls.h>


template<enum libcamera::ControlType T>
typename ControlTypeMap<T>::type
min(const libcamera::ControlValue &value);

template<enum libcamera::ControlType T>
typename ControlTypeMap<T>::type
max(const libcamera::ControlValue &value);

libcamera::ControlValue
clamp(const libcamera::ControlValue &value, const libcamera::ControlValue &min,
      const libcamera::ControlValue &max);

bool
operator<(const libcamera::ControlValue &a, const libcamera::ControlValue &b);

bool
operator>(const libcamera::ControlValue &a, const libcamera::ControlValue &b);
