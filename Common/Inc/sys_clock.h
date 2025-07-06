#ifndef __SYS_CLOCK_H__
#define __SYS_CLOCK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t getCurrentTime(void);
uint64_t getHighResTime_ns(void);

#ifdef __cplusplus
}
#endif

#endif // __SYS_CLOCK_H__
