#ifndef CYC_TASK_MULTI_H
#define CYC_TASK_MULTI_H

#include <kernel.h>

#define MAIN_PRIORITY 5
#ifndef STACK_SIZE
#define STACK_SIZE 4096
#endif

#ifndef TOPPERS_MACRO_ONLY
extern void Main(intptr_t exinf);
extern void count_task(intptr_t exinf);
extern void move_task(intptr_t exinf);
extern void test_spike_cychdr(intptr_t exinf);
#endif

#endif
