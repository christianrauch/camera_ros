#pragma once
#include <cstdint>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <string>

#define MAP(T, N)                                                   \
  template<>                                                        \
  struct ControlTypeMap<libcamera::details::control_type<T>::value> \
  {                                                                 \
    using type = T;                                                 \
  };                                                                \
  typedef ControlTypeMap<libcamera::ControlType##N>::type CT##N;


namespace std
{
std::string
to_string(const libcamera::ControlType id);
} // namespace std


// map 'ControlType' enums to C++ types

template<libcamera::ControlType>
struct ControlTypeMap;

MAP(void, None);
MAP(bool, Bool);
MAP(uint8_t, Byte);
MAP(int32_t, Integer32);
MAP(int64_t, Integer64);
MAP(float, Float);
MAP(std::string, String);
MAP(libcamera::Rectangle, Rectangle);
MAP(libcamera::Size, Size);
