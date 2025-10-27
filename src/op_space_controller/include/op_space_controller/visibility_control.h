#ifndef OP_SPACE_CONTROLLER__VISIBILITY_CONTROL_H_
#define OP_SPACE_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OP_SPACE_CONTROLLER_EXPORT __attribute__((dllexport))
    #define OP_SPACE_CONTROLLER_IMPORT __attribute__((dllimport))
  #else
    #define OP_SPACE_CONTROLLER_EXPORT __declspec(dllexport)
    #define OP_SPACE_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef OP_SPACE_CONTROLLER_BUILDING_DLL
    #define OP_SPACE_CONTROLLER_PUBLIC OP_SPACE_CONTROLLER_EXPORT
  #else
    #define OP_SPACE_CONTROLLER_PUBLIC OP_SPACE_CONTROLLER_IMPORT
  #endif
  #define OP_SPACE_CONTROLLER_PUBLIC_TYPE OP_SPACE_CONTROLLER_PUBLIC
  #define OP_SPACE_CONTROLLER_LOCAL
#else
  #define OP_SPACE_CONTROLLER_EXPORT __attribute__((visibility("default")))
  #define OP_SPACE_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define OP_SPACE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
    #define OP_SPACE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
  #else
    #define OP_SPACE_CONTROLLER_PUBLIC
    #define OP_SPACE_CONTROLLER_LOCAL
  #endif
  #define OP_SPACE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // OP_SPACE_CONTROLLER__VISIBILITY_CONTROL_H_

