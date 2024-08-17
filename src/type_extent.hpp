#pragma once
#include <cstddef>

namespace libcamera
{
class ControlId;
}

/**
 * @brief get the extent of a libcamera::ControlId
 * @param id
 * @return the extent of the control: 0 if the control is not a span,
 *         otherwise [1 ... libcamera::dynamic_extent]
 */
std::size_t
get_extent(const libcamera::ControlId *const id);
