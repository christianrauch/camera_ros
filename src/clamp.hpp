#pragma once
#include "cv_to_pv.hpp"
#include <libcamera/controls.h>


#define CASE_CLAMP(T)                                                                              \
  case libcamera::ControlType##T:                                                                  \
    return clamp<ControlTypeMap<libcamera::ControlType##T>::type>(value, min, max);

#define CASE_NONE(T)                                                                               \
  case libcamera::ControlType##T:                                                                  \
    return {};

namespace std
{

CTRectangle clamp(const CTRectangle &val, const CTRectangle &lo, const CTRectangle &hi)
{
  const int x = std::clamp(val.x, lo.x, hi.x);
  const int y = std::clamp(val.y, lo.y, hi.y);
  unsigned int width = std::clamp(x + val.width, lo.x + lo.width, hi.x + hi.width) - x;
  unsigned int height = std::clamp(y + val.height, lo.y + lo.height, hi.y + hi.height) - y;

  return CTRectangle {x, y, width, height};
}

} // namespace std


template<typename T>
libcamera::ControlValue clamp_array(const libcamera::ControlValue &value,
                                    const libcamera::ControlValue &min,
                                    const libcamera::ControlValue &max)
{
  const libcamera::Span<const T> v = value.get<libcamera::Span<const T>>();
  const libcamera::Span<const T> a = min.get<libcamera::Span<const T>>();
  const libcamera::Span<const T> b = max.get<libcamera::Span<const T>>();

  std::vector<T> vclamp(v.size());

  for (size_t i = 0; i < v.size(); i++)
    vclamp[i] = std::clamp(v[i], a[i], b[i]);

  return libcamera::ControlValue(libcamera::Span<const T>(vclamp));
}

template<typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<T>, CTBool>::value, bool> = true>
libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue &min,
                              const libcamera::ControlValue &max)
{
  if (value.isArray()) {
    return clamp_array<T>(value, min, max);
  }
  else {
    return std::clamp(value.get<T>(), min.get<T>(), max.get<T>());
    //    return clamp_scalar<T>(value, min, max);
  }
}

template<typename T,
         std::enable_if_t<std::is_same<std::remove_cv_t<T>, CTBool>::value, bool> = true>
libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue & /*min*/,
                              const libcamera::ControlValue & /*max*/)
{
  return value;
}

libcamera::ControlValue clamp(const libcamera::ControlValue &value,
                              const libcamera::ControlValue &min,
                              const libcamera::ControlValue &max)
{
  if (min.type() != max.type())
    throw std::runtime_error("minimum and maximum types mismatch");

  switch (value.type()) {
    CASE_NONE(None)
    CASE_CLAMP(Bool)
    CASE_CLAMP(Byte)
    CASE_CLAMP(Integer32)
    CASE_CLAMP(Integer64)
    CASE_CLAMP(Float)
    CASE_CLAMP(String)
    CASE_CLAMP(Rectangle)
    CASE_CLAMP(Size)
  }

  return {};
}

template<typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
T min(const libcamera::ControlValue &value)
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
T max(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    const libcamera::Span<const T> v = value.get<libcamera::Span<const T>>();
    return *std::max_element(v.begin(), v.end());
  }
  else {
    return value.get<T>();
  }
}
