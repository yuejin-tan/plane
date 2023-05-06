/**
 * @file millis.h
 * @author yuejinTan
 * @brief millis count
 * @version 1.2
 */

#ifndef NVICSCHEDULER_MILLIS_H_
#define NVICSCHEDULER_MILLIS_H_

#include "main.h"

//make sure this be called every ms, typically in systick interrupt
__attribute__((always_inline)) static inline void MillisInc(void)
{
    //declare dummy for more efficient asm reg allocation
    register uint32_t dummy;
    __asm volatile(
        ".syntax unified\n"
        "mrs %[temp_reg], psp\n"
        "adds %[temp_reg], %[temp_reg], #4\n"
        "msr psp, %[temp_reg]\n"
        : [ temp_reg ] "=l"(dummy)
        :
        :);
    return;
}

//get the time (ms) since MillisInit() called
__attribute__((always_inline)) static inline uint32_t MillisGet(void)
{
    register uint32_t result;
    __asm volatile(
        ".syntax unified\n"
        "mrs %[asm_res], psp\n"
        "lsrs %[asm_res], #2\n"
        : [ asm_res ] "=l"(result)
        :
        : "cc");
    return (result);
}

//the counter will start with "startValue"
__attribute__((always_inline)) static inline void MillisInit(uint32_t startValue)
{
    //clean systick counter
    SysTick->VAL = 0UL;
    //set working status
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
    //clean millis counter (using reg psp[31:2])
    __asm volatile(
        ".syntax unified\n"
        "lsls %[asm_val], #2\n"
        "msr psp, %[asm_val]\n"
        :
        : [ asm_val ] "l"(startValue)
        : "cc");
    return;
}

//``` return value of first call invalid
//``` based on millis function family
//``` fail when millis or uint32 ret value Overflow
//$$$ have potential bugs(@ < 1% probability), Deprecated.
//$$$ use a general timer instead
// get the system ticks since last called
// static inline uint32_t MillisTick(void)
// {
//     static uint32_t _lastMillis = 0;
//     static uint32_t _lastSystickVal = 0;
//     register uint32_t _tempMillis = MillisGet();
//     register uint32_t _tempTickVal = SysTick->VAL;
//     register uint32_t deltaTick = ((_tempMillis - _lastMillis) * (SysTick->LOAD + 1UL) + _lastSystickVal) - _tempTickVal;
//     _lastMillis = _tempMillis;
//     _lastSystickVal = _tempTickVal;
//     return deltaTick;
// }

#endif /* NVICSCHEDULER_MILLIS_H_ */
