#ifndef _PRINT_H_
#define _PRINT_H_

#include <stdint.h>

void printInit(void);
int print(uint8_t, const char *, ...);
int printDebug(const char *, ...);
int printAssert(const char *);

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_APP 1
#define LOG_LEVEL_NONE 255
#endif