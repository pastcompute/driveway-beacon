#ifndef DRIVEWAY_MONITOR_DEBUG_H__
#define DRIVEWAY_MONITOR_DEBUG_H__

#define VERBOSE 0

#include <stdio.h>

#if VERBOSE
#define DEBUG(x ...) { char buf[128]; snprintf(buf, sizeof(buf), x); Serial.print(buf); }
#else
#define DEBUG(x ...)
#endif

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

#endif
