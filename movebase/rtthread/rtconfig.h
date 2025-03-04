/* RT-Thread config file */

#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

#define RT_USING_LIBC
#define RT_THREAD_PRIORITY_MAX  32
// 1ms
#define RT_TICK_PER_SECOND  1000
// align size for CPU architecture data access
#define RT_ALIGN_SIZE   4
// the max length of object name <2-16>
#define RT_NAME_MAX     16
// Using RT-Thread components initialization
#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
// the stack size of main thread <1-4086>
#define RT_MAIN_THREAD_STACK_SIZE     1024
#define RT_USING_OVERFLOW_CHECK
// Software timers Configuration
//  Enables user timers
#define RT_USING_TIMER_SOFT         0
#if RT_USING_TIMER_SOFT == 0
    #undef RT_USING_TIMER_SOFT
#endif
// The priority level of timer thread <0-31>
#define RT_TIMER_THREAD_PRIO        4
// The stack size of timer thread <0-8192>
// Default: 512
#define RT_TIMER_THREAD_STACK_SIZE  512

// IPC
#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
//#define RT_USING_EVENT
//#define RT_USING_MAILBOX
//#define RT_USING_MESSAGEQUEUE

// MM
//#define RT_USING_MEMPOOL
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM
//#define RT_USING_MEMHEAP
//#define RT_MEMHEAP_FAST_MODE
#define RT_USING_SMALL_MEM_AS_HEAP
//#define RT_USING_TINY_SIZE


// rt_kprintf
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE  128


#endif