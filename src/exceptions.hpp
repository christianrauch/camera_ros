#pragma once
#include <libcamera/controls.h>
#include <stdexcept>


class invalid_conversion : public std::runtime_error
{
public:
  explicit invalid_conversion(const std::string &msg) : std::runtime_error(msg) {}
};

class should_not_reach : public std::runtime_error
{
public:
  explicit should_not_reach() : std::runtime_error("should not reach here") {}
};

class unknown_control : public std::runtime_error
{
public:
  explicit unknown_control(const libcamera::ControlId *const id) : std::runtime_error("unknown control: " + id->name() + " (" + std::to_string(id->id()) + ")") {}
};

class unsupported_control : public std::runtime_error
{
public:
  explicit unsupported_control(const libcamera::ControlId *const id) : std::runtime_error("unsupported control: " + id->name() + " (" + std::to_string(id->id()) + ")") {}
};
