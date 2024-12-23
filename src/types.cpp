#include "types.hpp"
#include <libcamera/controls.h>


#define CASE_TYPE(T)              \
  case libcamera::ControlType##T: \
    return #T;

std::string
std::to_string(const libcamera::ControlType id)
{
  switch (id) {
    CASE_TYPE(None)
    CASE_TYPE(Bool)
    CASE_TYPE(Byte)
    CASE_TYPE(Integer32)
    CASE_TYPE(Integer64)
    CASE_TYPE(Float)
    CASE_TYPE(String)
    CASE_TYPE(Rectangle)
    CASE_TYPE(Size)
#if LIBCAMERA_VER_GE(0, 4, 0)
    CASE_TYPE(Unsigned16)
    CASE_TYPE(Unsigned32)
    CASE_TYPE(Point)
#endif
  }

  return {};
}
