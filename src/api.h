#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SampleCommunicationController_DLLIMPORT __declspec(dllimport)
#  define SampleCommunicationController_DLLEXPORT __declspec(dllexport)
#  define SampleCommunicationController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define SampleCommunicationController_DLLIMPORT __attribute__((visibility("default")))
#    define SampleCommunicationController_DLLEXPORT __attribute__((visibility("default")))
#    define SampleCommunicationController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define SampleCommunicationController_DLLIMPORT
#    define SampleCommunicationController_DLLEXPORT
#    define SampleCommunicationController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef SampleCommunicationController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define SampleCommunicationController_DLLAPI
#  define SampleCommunicationController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef SampleCommunicationController_EXPORTS
#    define SampleCommunicationController_DLLAPI SampleCommunicationController_DLLEXPORT
#  else
#    define SampleCommunicationController_DLLAPI SampleCommunicationController_DLLIMPORT
#  endif // SampleCommunicationController_EXPORTS
#  define SampleCommunicationController_LOCAL SampleCommunicationController_DLLLOCAL
#endif // SampleCommunicationController_STATIC