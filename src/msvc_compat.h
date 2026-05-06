#ifndef GRBLHAL_SIM_MSVC_COMPAT_H
#define GRBLHAL_SIM_MSVC_COMPAT_H

#ifdef _MSC_VER

#ifndef __attribute__
#define __attribute__(x)
#endif

#ifndef __time_t_defined
#define __time_t_defined
#endif

#ifndef strcasecmp
#define strcasecmp _stricmp
#endif

#ifndef strncasecmp
#define strncasecmp _strnicmp
#endif

#ifndef strdup
#define strdup _strdup
#endif

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#endif

#endif
