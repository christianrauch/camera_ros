#pragma once
#include <libcamera/version.h>

#define LIBCAMERA_VER_GE(major, minor, patch)                               \
  ((major < LIBCAMERA_VERSION_MAJOR) ||                                     \
   (major == LIBCAMERA_VERSION_MAJOR && minor < LIBCAMERA_VERSION_MINOR) || \
   (major == LIBCAMERA_VERSION_MAJOR && minor == LIBCAMERA_VERSION_MINOR && \
    patch <= LIBCAMERA_VERSION_PATCH))
