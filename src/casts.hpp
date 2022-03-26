#pragma once
#include "types.hpp"
#include <sstream>


template<typename T, typename F, std::enable_if_t<std::is_same<T, F>::value, bool> = true>
const T &
cast(const F &value)
{
  return value;
}

template<typename T, typename F, std::enable_if_t<!std::is_same<T, F>::value, bool> = true>
T
cast(const F &value)
{
  return T(value);
}

// from ControlTypeBool

template<>
CTString
cast(const CTBool &value)
{
  return std::to_string(value);
}

// from ControlTypeByte

template<>
CTString
cast(const CTByte &value)
{
  return std::to_string(value);
}

// from ControlTypeInteger32

template<>
CTString
cast(const CTInteger32 &value)
{
  return std::to_string(value);
}

// from ControlTypeInteger64

template<>
CTString
cast(const CTInteger64 &value)
{
  return std::to_string(value);
}

// from ControlTypeFloat

template<>
CTString
cast(const CTFloat &value)
{
  return std::to_string(value);
}

// from ControlTypeString

template<>
CTBool
cast(const CTString &value)
{
  bool v;
  std::istringstream vss(value);

  vss >> std::boolalpha >> v;

  if (vss.fail())
    throw std::invalid_argument("invalid string representation for boolean: '" + value + "'");

  return v;
}

template<>
CTByte
cast(const CTString &value)
{
  return std::stoi(value);
}

template<>
CTInteger32
cast(const CTString &value)
{
  // long int
  return std::stol(value);
}

template<>
CTInteger64
cast(const CTString &value)
{
  // long long int
  return std::stoll(value);
}

template<>
CTFloat
cast(const CTString &value)
{
  return std::stof(value);
}
