#ifndef STEPPER_JTC_CONTROL__VISIBILITY_CONTROL_H_
#define STEPPER_JTC_CONTROL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STEPPER_CONTROL_EXPORT __attribute__ ((dllexport))
    #define STEPPER_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define STEPPER_CONTROL_EXPORT __declspec(dllexport)
    #define STEPPER_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef STEPPER_CONTROL_BUILDING_DLL
    #define STEPPER_CONTROL_PUBLIC STEPPER_CONTROL_EXPORT
  #else
    #define STEPPER_CONTROL_PUBLIC STEPPER_CONTROL_IMPORT
  #endif
  #define STEPPER_CONTROL_PUBLIC_TYPE STEPPER_CONTROL_PUBLIC
  #define STEPPER_CONTROL_LOCAL
#else
  #define STEPPER_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define STEPPER_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define STEPPER_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define STEPPER_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STEPPER_CONTROL_PUBLIC
    #define STEPPER_CONTROL_LOCAL
  #endif
  #define STEPPER_CONTROL_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // STEPPER_JTC_CONTROL__VISIBILITY_CONTROL_H_
