#ifndef ROBOCLAW_HARDWARE__VISIBILITY_CONTROL_H_
#define ROBOCLAW_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOCLAW_HARDWARE_EXPORT __attribute__((dllexport))
    #define ROBOCLAW_HARDWARE_IMPORT __attribute__((dllimport))
  #else
    #define ROBOCLAW_HARDWARE_EXPORT __declspec(dllexport)
    #define ROBOCLAW_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOCLAW_HARDWARE_BUILDING_DLL
    #define ROBOCLAW_HARDWARE_PUBLIC ROBOCLAW_HARDWARE_EXPORT
  #else
    #define ROBOCLAW_HARDWARE_PUBLIC ROBOCLAW_HARDWARE_IMPORT
  #endif
  #define ROBOCLAW_HARDWARE_LOCAL
#else
  #define ROBOCLAW_HARDWARE_EXPORT __attribute__((visibility("default")))
  #define ROBOCLAW_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define ROBOCLAW_HARDWARE_PUBLIC __attribute__((visibility("default")))
    #define ROBOCLAW_HARDWARE_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define ROBOCLAW_HARDWARE_PUBLIC
    #define ROBOCLAW_HARDWARE_LOCAL
  #endif
#endif

#endif  // ROBOCLAW_HARDWARE__VISIBILITY_CONTROL_H_
