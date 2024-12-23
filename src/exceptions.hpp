#pragma once
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
