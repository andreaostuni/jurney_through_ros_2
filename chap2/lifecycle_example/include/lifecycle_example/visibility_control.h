#ifndef LIFECYCLE_EXAMPLE__VISIBILITY_CONTROL_H_
#define LIFECYCLE_EXAMPLE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIFECYCLE_EXAMPLE_EXPORT __attribute__ ((dllexport))
    #define LIFECYCLE_EXAMPLE_IMPORT __attribute__ ((dllimport))
  #else
    #define LIFECYCLE_EXAMPLE_EXPORT __declspec(dllexport)
    #define LIFECYCLE_EXAMPLE_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIFECYCLE_EXAMPLE_BUILDING_LIBRARY
    #define LIFECYCLE_EXAMPLE_PUBLIC LIFECYCLE_EXAMPLE_EXPORT
  #else
    #define LIFECYCLE_EXAMPLE_PUBLIC LIFECYCLE_EXAMPLE_IMPORT
  #endif
  #define LIFECYCLE_EXAMPLE_PUBLIC_TYPE LIFECYCLE_EXAMPLE_PUBLIC
  #define LIFECYCLE_EXAMPLE_LOCAL
#else
  #define LIFECYCLE_EXAMPLE_EXPORT __attribute__ ((visibility("default")))
  #define LIFECYCLE_EXAMPLE_IMPORT
  #if __GNUC__ >= 4
    #define LIFECYCLE_EXAMPLE_PUBLIC __attribute__ ((visibility("default")))
    #define LIFECYCLE_EXAMPLE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIFECYCLE_EXAMPLE_PUBLIC
    #define LIFECYCLE_EXAMPLE_LOCAL
  #endif
  #define LIFECYCLE_EXAMPLE_PUBLIC_TYPE
#endif

#endif  // LIFECYCLE_EXAMPLE__VISIBILITY_CONTROL_H_
