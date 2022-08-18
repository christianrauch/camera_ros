#pragma once
#include <libcamera/controls.h>


namespace std
{
std::string
to_string(const libcamera::ControlType id);
} // namespace std


// map 'ControlType' enums to C++ types

template<libcamera::ControlType>
struct ControlTypeMap;

template<>
struct ControlTypeMap<libcamera::ControlTypeNone>
{
  using type = void;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeBool>
{
  using type = bool;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeByte>
{
  using type = uint8_t;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeInteger32>
{
  using type = int32_t;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeInteger64>
{
  using type = int64_t;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeFloat>
{
  using type = float;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeString>
{
  using type = std::string;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeRectangle>
{
  using type = libcamera::Rectangle;
};

template<>
struct ControlTypeMap<libcamera::ControlTypeSize>
{
  using type = libcamera::Size;
};

typedef ControlTypeMap<libcamera::ControlTypeNone>::type CTNone;
typedef ControlTypeMap<libcamera::ControlTypeBool>::type CTBool;
typedef ControlTypeMap<libcamera::ControlTypeByte>::type CTByte;
typedef ControlTypeMap<libcamera::ControlTypeInteger32>::type CTInteger32;
typedef ControlTypeMap<libcamera::ControlTypeInteger64>::type CTInteger64;
typedef ControlTypeMap<libcamera::ControlTypeFloat>::type CTFloat;
typedef ControlTypeMap<libcamera::ControlTypeString>::type CTString;
typedef ControlTypeMap<libcamera::ControlTypeRectangle>::type CTRectangle;
typedef ControlTypeMap<libcamera::ControlTypeSize>::type CTSize;
