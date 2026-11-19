#ifndef CYC_TASK_REALTIME_H
#define CYC_TASK_REALTIME_H

#include <kernel.h>

#ifndef STACK_SIZE
#define STACK_SIZE 4096
#endif

#ifndef TOPPERS_MACRO_ONLY
extern void Main(intptr_t exinf);
extern void sound_task(intptr_t exinf);
#endif

#endif
