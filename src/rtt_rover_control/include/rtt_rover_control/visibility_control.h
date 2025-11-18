#ifndef RTT_ROVER_CONTROL__VISIBILITY_CONTROL_H_
#define RTT_ROVER_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RTT_ROVER_CONTROL_EXPORT __attribute__ ((dllexport))
    #define RTT_ROVER_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define RTT_ROVER_CONTROL_EXPORT __declspec(dllexport)
    #define RTT_ROVER_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef RTT_ROVER_CONTROL_BUILDING_LIBRARY
    #define RTT_ROVER_CONTROL_PUBLIC RTT_ROVER_CONTROL_EXPORT
  #else
    #define RTT_ROVER_CONTROL_PUBLIC RTT_ROVER_CONTROL_IMPORT
  #endif
  #define RTT_ROVER_CONTROL_PUBLIC_TYPE RTT_ROVER_CONTROL_PUBLIC
  #define RTT_ROVER_CONTROL_LOCAL
#else
  #define RTT_ROVER_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define RTT_ROVER_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define RTT_ROVER_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define RTT_ROVER_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RTT_ROVER_CONTROL_PUBLIC
    #define RTT_ROVER_CONTROL_LOCAL
  #endif
  #define RTT_ROVER_CONTROL_PUBLIC_TYPE
#endif

#endif  // RTT_ROVER_CONTROL__VISIBILITY_CONTROL_H_
