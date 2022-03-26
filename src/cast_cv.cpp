#include "cast_cv.hpp"
#include "casts.hpp"


#define CASE_CAST_CONV1(F)                                                                         \
  case libcamera::ControlType##F:                                                                  \
    return cast_cv<ControlTypeMap<libcamera::ControlType##F>::type>(value);

#define CASE_CAST_CONV2(F)                                                                         \
  case libcamera::ControlType##F:                                                                  \
    return cast_cv<ControlTypeMap<libcamera::ControlType##F>::type, T>(value);

#define CASE_NONE(T)                                                                               \
  case libcamera::ControlType##T:                                                                  \
    return {};


template<typename F, typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value &&
                            !std::is_same<std::remove_cv_t<T>, CTBool>::value &&
                            !(std::is_same<std::remove_cv_t<F>, CTRectangle>::value ||
                              std::is_same<std::remove_cv_t<T>, CTRectangle>::value ||
                              std::is_same<std::remove_cv_t<F>, CTSize>::value ||
                              std::is_same<std::remove_cv_t<T>, CTSize>::value),
                          bool> = true>
libcamera::ControlValue
cast_cv(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    std::vector<T> a;
    for (const F &v : value.get<libcamera::Span<const F>>())
      a.push_back(cast<T, F>(v));
    return libcamera::ControlValue(libcamera::Span<const T>(a));
  }
  else {
    return libcamera::ControlValue(cast<T, F>(value.get<F>()));
  }
}

template<typename F, typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value &&
                            std::is_same<std::remove_cv_t<T>, CTBool>::value &&
                            !(std::is_same<std::remove_cv_t<F>, CTRectangle>::value ||
                              std::is_same<std::remove_cv_t<F>, CTSize>::value),
                          bool> = true>
libcamera::ControlValue
cast_cv(const libcamera::ControlValue &value)
{
  if (value.isArray())
    throw std::runtime_error("unsupported array conversion for CTBool");
  else
    return libcamera::ControlValue(cast<T, F>(value.get<F>()));
}

template<
  typename F, typename T,
  std::enable_if_t<std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value, bool> = true>
const libcamera::ControlValue &
cast_cv(const libcamera::ControlValue &value)
{
  return value;
}

template<typename F, typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value &&
                            (std::is_same<std::remove_cv_t<F>, CTRectangle>::value ||
                             std::is_same<std::remove_cv_t<T>, CTRectangle>::value ||
                             std::is_same<std::remove_cv_t<F>, CTSize>::value ||
                             std::is_same<std::remove_cv_t<T>, CTSize>::value),
                          bool> = true>
libcamera::ControlValue
cast_cv(const libcamera::ControlValue & /*value*/)
{
  throw std::runtime_error("unsupported ControlValue cast");
}

template<typename T>
libcamera::ControlValue
cast_cv(const libcamera::ControlValue &value)
{
  switch (value.type()) {
    CASE_NONE(None)
    CASE_CAST_CONV2(Bool)
    CASE_CAST_CONV2(Byte)
    CASE_CAST_CONV2(Integer32)
    CASE_CAST_CONV2(Integer64)
    CASE_CAST_CONV2(Float)
    CASE_CAST_CONV2(String)
    CASE_CAST_CONV2(Rectangle)
    CASE_CAST_CONV2(Size)
  }

  return {};
}

libcamera::ControlValue
cast_cv(const libcamera::ControlValue &value, const libcamera::ControlType target_type)
{
  switch (target_type) {
    CASE_NONE(None)
    CASE_CAST_CONV1(Bool)
    CASE_CAST_CONV1(Byte)
    CASE_CAST_CONV1(Integer32)
    CASE_CAST_CONV1(Integer64)
    CASE_CAST_CONV1(Float)
    CASE_CAST_CONV1(String)
    CASE_CAST_CONV1(Rectangle)
    CASE_CAST_CONV1(Size)
  }

  return {};
}
