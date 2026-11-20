#ifndef PID_LINE_TRACER_H
#define PID_LINE_TRACER_H

#include <kernel.h>

/* タスクの優先度 */
#define MAIN_PRIORITY 5

#ifndef STACK_SIZE
#define STACK_SIZE 4096
#endif

#ifndef TOPPERS_MACRO_ONLY
extern void Main(intptr_t exinf);
#endif

#endif
