#pragma once
#include <any>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
//#include <variant>
#include <sstream>

//bool is_integer(const libcamera::ControlType &type)
//{
//  return (type == libcamera::ControlTypeInteger32) || (type == libcamera::ControlTypeInteger64);
//}

//bool is_float(const libcamera::ControlType &type)
//{
//  return (type == libcamera::ControlTypeFloat);
//}


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

std::any convert_type(const libcamera::ControlValue &value_source)
{
  switch (value_source.type()) {
  case libcamera::ControlTypeNone:
    break;
  case libcamera::ControlTypeBool:
    return value_source.get<CTBool>();
    break;
  case libcamera::ControlTypeByte:
    return value_source.get<CTByte>();
    break;
  case libcamera::ControlTypeInteger32:
    return value_source.get<CTInteger32>();
    break;
  case libcamera::ControlTypeInteger64:
    return value_source.get<CTInteger64>();
    break;
  case libcamera::ControlTypeFloat:
    return value_source.get<CTFloat>();
    break;
  case libcamera::ControlTypeString:
    return value_source.get<CTString>();
    break;
  case libcamera::ControlTypeRectangle:
    return value_source.get<CTRectangle>();
    break;
  case libcamera::ControlTypeSize:
    return value_source.get<CTSize>();
    break;
  }
  return {};
}

template<typename T, typename F>
T convert(const F &value)
{
  return T(value);
}

// from ControlTypeBool

template<>
CTString convert(const CTBool &value)
{
  return std::to_string(value);
}

// from ControlTypeByte

template<>
CTString convert(const CTByte &value)
{
  return std::to_string(value);
}

// from ControlTypeInteger32

template<>
CTString convert(const CTInteger32 &value)
{
  return std::to_string(value);
}

// from ControlTypeInteger64

template<>
CTString convert(const CTInteger64 &value)
{
  return std::to_string(value);
}

// from ControlTypeFloat

template<>
CTString convert(const CTFloat &value)
{
  return std::to_string(value);
}

// from ControlTypeString

template<>
CTBool convert(const CTString &value)
{
  bool v;
  std::istringstream vss(value);

  vss >> std::boolalpha >> v;

  if (vss.fail())
    throw std::invalid_argument("invalid string representation for boolean: '" + value + "'");

  return v;
}

template<>
CTByte convert(const CTString &value)
{
  return std::stoi(value);
}

template<>
CTInteger32 convert(const CTString &value)
{
  // long int
  return std::stol(value);
}

template<>
CTInteger64 convert(const CTString &value)
{
  // long long int
  return std::stoll(value);
}

template<>
CTFloat convert(const CTString &value)
{
  return std::stof(value);
}

template<typename T, typename F>
T convert_any(const std::any &value)
{
  return convert<T, F>(std::any_cast<F>(value));
}

template<typename T>
std::any cast_type(const std::any &value)
{
  if (typeid(T) == value.type())
    return value;

  if (value.type() == typeid(CTNone))
    return {};
  else if (value.type() == typeid(CTBool))
    return convert_any<T, CTBool>(value);
  else if (value.type() == typeid(CTByte))
    return convert_any<T, CTByte>(value);
  else if (value.type() == typeid(CTInteger32))
    return convert_any<T, CTInteger32>(value);
  else if (value.type() == typeid(CTInteger64))
    return convert_any<T, CTInteger64>(value);
  else if (value.type() == typeid(CTFloat))
    return convert_any<T, CTFloat>(value);
  else if (value.type() == typeid(CTString))
    return convert_any<T, CTString>(value);
  else if (value.type() == typeid(CTRectangle))
    return {};
  else if (value.type() == typeid(CTSize))
    return {};
  else
    return {};

  return value;
}

//ControlTypeMap<libcamera::ControlTypeByte>::type & ::operator=(
//  const ControlTypeMap<libcamera::ControlTypeRectangle>::type &other)
//{
//  //
//}

//operator ControlTypeMap<libcamera::ControlTypeByte>::type(
//  const ControlTypeMap<libcamera::ControlTypeRectangle>::type &other)
//{
//  //
//}

//template<ControlTypeMap<libcamera::ControlTypeByte>::type>
//bool cast_type(std::any &value)
//{
//  if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeNone>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeNone>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeBool>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeBool>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeByte>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeByte>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeInteger32>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeInteger32>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeInteger64>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeInteger64>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeFloat>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeFloat>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeString>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeString>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeRectangle>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeRectangle>::type>(value));
//  }
//  else if (value.type() == typeid(ControlTypeMap<libcamera::ControlTypeSize>::type)) {
//    value = T(std::any_cast<ControlTypeMap<libcamera::ControlTypeSize>::type>(value));
//  }

//  return true;
//}
