#ifndef VIMBAX_CAMERA_EVENTS__VISIBILITY_CONTROL_H_
#define VIMBAX_CAMERA_EVENTS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIMBAX_CAMERA_EVENTS_EXPORT __attribute__ ((dllexport))
    #define VIMBAX_CAMERA_EVENTS_IMPORT __attribute__ ((dllimport))
  #else
    #define VIMBAX_CAMERA_EVENTS_EXPORT __declspec(dllexport)
    #define VIMBAX_CAMERA_EVENTS_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIMBAX_CAMERA_EVENTS_BUILDING_LIBRARY
    #define VIMBAX_CAMERA_EVENTS_PUBLIC VIMBAX_CAMERA_EVENTS_EXPORT
  #else
    #define VIMBAX_CAMERA_EVENTS_PUBLIC VIMBAX_CAMERA_EVENTS_IMPORT
  #endif
  #define VIMBAX_CAMERA_EVENTS_PUBLIC_TYPE VIMBAX_CAMERA_EVENTS_PUBLIC
  #define VIMBAX_CAMERA_EVENTS_LOCAL
#else
  #define VIMBAX_CAMERA_EVENTS_EXPORT __attribute__ ((visibility("default")))
  #define VIMBAX_CAMERA_EVENTS_IMPORT
  #if __GNUC__ >= 4
    #define VIMBAX_CAMERA_EVENTS_PUBLIC __attribute__ ((visibility("default")))
    #define VIMBAX_CAMERA_EVENTS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIMBAX_CAMERA_EVENTS_PUBLIC
    #define VIMBAX_CAMERA_EVENTS_LOCAL
  #endif
  #define VIMBAX_CAMERA_EVENTS_PUBLIC_TYPE
#endif

#endif  // VIMBAX_CAMERA_EVENTS__VISIBILITY_CONTROL_H_
