#ifndef LED_H
#define LED_H
#include <kernel.h>
#define MAIN_PRIORITY 5
#define MAIN_STACK_SIZE 4096
#define STACK_SIZE MAIN_STACK_SIZE
#ifndef TOPPERS_MACRO_ONLY
extern void main_task(intptr_t exinf);
#endif
#endif
