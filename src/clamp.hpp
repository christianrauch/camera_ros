#pragma once
#include <libcamera/controls.h>


template<typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
T
min(const libcamera::ControlValue &value)
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
T
max(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    const libcamera::Span<const T> v = value.get<libcamera::Span<const T>>();
    return *std::max_element(v.begin(), v.end());
  }
  else {
    return value.get<T>();
  }
}

libcamera::ControlValue
clamp(const libcamera::ControlValue &value, const libcamera::ControlValue &min,
      const libcamera::ControlValue &max);

bool
operator<(const libcamera::ControlValue &a, const libcamera::ControlValue &b);

bool
operator>(const libcamera::ControlValue &a, const libcamera::ControlValue &b);
