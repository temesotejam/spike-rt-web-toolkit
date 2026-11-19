#include <kernel.h>

/* タスクの優先度 */
#define MAIN_PRIORITY 5

#ifndef TASK_PORTID
#define TASK_PORTID 1
#endif

#define SIO_USART_F_PORTID    1
#define SIO_USB_PORTID        2
#define SIO_BLUETOOTH_PORTID  3
#define SIO_TEST_PORTID       4

#ifndef STACK_SIZE
#define STACK_SIZE 4096
#endif

#ifndef TOPPERS_MACRO_ONLY
extern void Main(intptr_t exinf);
extern void main_task(intptr_t exinf);
extern void sub_task(intptr_t exinf);
#endif
