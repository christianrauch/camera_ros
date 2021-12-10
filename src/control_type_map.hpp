#pragma once
#include <any>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
//#include <variant>

bool is_integer(const libcamera::ControlType &type)
{
  return (type == libcamera::ControlTypeInteger32) || (type == libcamera::ControlTypeInteger64);
}

bool is_float(const libcamera::ControlType &type)
{
  return (type == libcamera::ControlTypeFloat);
}


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


/**
 * @brief variant with all possible converted types
 */
//typedef std::variant<bool, uint8_t, int32_t, int64_t, float, std::string, libcamera::Rectangle,
//                     libcamera::Size>
//  control_type_variant;

/**
 * @brief convert a control value to given type
 * @param value_source source value
 * @param type_target target type
 * @return std::variant
 */
//control_type_variant
//convert_type(const libcamera::ControlValue &value_source,
//             const libcamera::ControlType &type_target = libcamera::ControlTypeNone)
//{
//  control_type_variant value;

//  switch (value_source.type()) {
//  case libcamera::ControlTypeNone:
//    break;
//  case libcamera::ControlTypeBool:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeBool>::type>();
//    break;
//  case libcamera::ControlTypeByte:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeByte>::type>();
//    break;
//  case libcamera::ControlTypeInteger32:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeInteger32>::type>();
//    break;
//  case libcamera::ControlTypeInteger64:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeInteger64>::type>();
//    break;
//  case libcamera::ControlTypeFloat:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeFloat>::type>();
//    break;
//  case libcamera::ControlTypeString:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeString>::type>();
//    break;
//  case libcamera::ControlTypeRectangle:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeRectangle>::type>();
//    break;
//  case libcamera::ControlTypeSize:
//    value = value_source.get<ControlTypeMap<libcamera::ControlTypeSize>::type>();
//    break;
//  }

//  switch (type_target) {
//  case libcamera::ControlTypeNone:
//    return value;
//  case libcamera::ControlTypeBool:
//    return std::get<ControlTypeMap<libcamera::ControlTypeBool>::type>(value);
//  case libcamera::ControlTypeByte:
//    return std::get<ControlTypeMap<libcamera::ControlTypeByte>::type>(value);
//  case libcamera::ControlTypeInteger32:
//    return std::get<ControlTypeMap<libcamera::ControlTypeInteger32>::type>(value);
//  case libcamera::ControlTypeInteger64:
//    return std::get<ControlTypeMap<libcamera::ControlTypeInteger64>::type>(value);
//  case libcamera::ControlTypeFloat:
//    return std::get<ControlTypeMap<libcamera::ControlTypeFloat>::type>(value);
//  case libcamera::ControlTypeString:
//    return std::get<ControlTypeMap<libcamera::ControlTypeString>::type>(value);
//  case libcamera::ControlTypeRectangle:
//    return std::get<ControlTypeMap<libcamera::ControlTypeRectangle>::type>(value);
//  case libcamera::ControlTypeSize:
//    return std::get<ControlTypeMap<libcamera::ControlTypeSize>::type>(value);
//  }
//}

std::any convert_type2(const libcamera::ControlValue &value_source/*,
                       const libcamera::ControlType &type_target = libcamera::ControlTypeNone*/)
{
  std::any value;

  switch (value_source.type()) {
  case libcamera::ControlTypeNone:
    break;
  case libcamera::ControlTypeBool:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeBool>::type>();
    break;
  case libcamera::ControlTypeByte:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeByte>::type>();
    break;
  case libcamera::ControlTypeInteger32:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeInteger32>::type>();
    break;
  case libcamera::ControlTypeInteger64:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeInteger64>::type>();
    break;
  case libcamera::ControlTypeFloat:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeFloat>::type>();
    break;
  case libcamera::ControlTypeString:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeString>::type>();
    break;
  case libcamera::ControlTypeRectangle:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeRectangle>::type>();
    break;
  case libcamera::ControlTypeSize:
    value = value_source.get<ControlTypeMap<libcamera::ControlTypeSize>::type>();
    break;
  }

  return value;

  //  switch (type_target) {
  //  case libcamera::ControlTypeNone:
  //    return value;
  //  case libcamera::ControlTypeBool:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeBool>::type>(value);
  //  case libcamera::ControlTypeByte:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeByte>::type>(value);
  //  case libcamera::ControlTypeInteger32:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeInteger32>::type>(value);
  //  case libcamera::ControlTypeInteger64:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeInteger64>::type>(value);
  //  case libcamera::ControlTypeFloat:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeFloat>::type>(value);
  //  case libcamera::ControlTypeString:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeString>::type>(value);
  //  case libcamera::ControlTypeRectangle:
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeRectangle>::type>(value);
  //  case libcamera::ControlTypeSize:;
  //    return std::any_cast<ControlTypeMap<libcamera::ControlTypeSize>::type>(value);
  //  }
}
