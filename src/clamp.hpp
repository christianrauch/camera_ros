#pragma once
#include "types.hpp"
#include <libcamera/controls.h>


template<enum libcamera::ControlType T>
typename ControlTypeMap<T>::type
min(const libcamera::ControlValue &value)
{
  using A = typename ControlTypeMap<T>::type;
  using S = libcamera::Span<const A>;

  if (value.isArray()) {
    const S v = value.get<S>();
    return *std::min_element(v.begin(), v.end());
  }
  else {
    return value.get<A>();
  }
}

template<enum libcamera::ControlType T>
typename ControlTypeMap<T>::type
max(const libcamera::ControlValue &value)
{
  using A = typename ControlTypeMap<T>::type;
  using S = libcamera::Span<const A>;

  if (value.isArray()) {
    const S v = value.get<S>();
    return *std::max_element(v.begin(), v.end());
  }
  else {
    return value.get<A>();
  }
}

libcamera::ControlValue
clamp(const libcamera::ControlValue &value, const libcamera::ControlValue &min,
      const libcamera::ControlValue &max);

bool
operator<(const libcamera::ControlValue &a, const libcamera::ControlValue &b);

bool
operator>(const libcamera::ControlValue &a, const libcamera::ControlValue &b);
