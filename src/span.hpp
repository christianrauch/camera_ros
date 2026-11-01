#pragma once

#include "libcamera_version_utils.hpp"


#if LIBCAMERA_VER_GE(0, 8, 0)

#include <span>

template<typename T>
using cspan = std::span<T>;

constexpr size_t cdynamic_extent = std::dynamic_extent;

#else

#include <libcamera/base/span.h>

template<typename T>
using cspan = libcamera::Span<T>;

constexpr size_t cdynamic_extent = libcamera::dynamic_extent;

#endif
