#pragma once
#include <cstddef>

namespace libcamera
{
class ControlId;
}

std::size_t
get_extent(const libcamera::ControlId *id);
