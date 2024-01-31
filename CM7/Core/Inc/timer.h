/*
    This file contains functions for handling real-time timers.
*/
// timer.h
#ifndef TIMER_H
#define TIMER_H

#include "stm32h7xx_hal.h"
#define MAX_TIMERS 1  // Adjust based on the number of timers you have

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t index;
} Timer;


void timer_start(Timer *timer);
float timer_GetElapsedTimeMicro(Timer *timer);
void timer_stop(Timer *timer);

#endif // TIMER_H
