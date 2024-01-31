// timer.c
#include "timer.h"
#include "main.h"

// Array to hold the overflow counts for each timer

// with 64 bits we have much time befeore overflow
// Even with microsecond resolution on our clock
// 2**64 * 0.000001 / 1000 / 1000 / 60 / 60 / 24 / 365 = 0.5849424173550719 years
volatile static uint64_t overflowCounts[MAX_TIMERS] = {0};

// Start a timer and reset its overflow count
void timer_start(Timer *timer) {
    overflowCounts[timer->index] = 0; // Reset overflow count for this timer
    __HAL_TIM_SET_COUNTER(timer->htim, 0);  // Reset the counter
    __HAL_TIM_CLEAR_FLAG(timer->htim, TIM_FLAG_UPDATE); // Clear the update flag
    HAL_TIM_Base_Start_IT(timer->htim);    // Start the timer with interrupt
}
// Start a timer and reset its overflow count
void timer_stop(Timer *timer) {
    HAL_TIM_Base_Stop_IT(timer->htim);    // Start the timer with interrupt
    overflowCounts[timer->index] = 0; // Reset overflow count for this timer
    __HAL_TIM_SET_COUNTER(timer->htim, 0);  // Reset the counter
    __HAL_TIM_CLEAR_FLAG(timer->htim, TIM_FLAG_UPDATE); // Clear the update flag
}

// Get the elapsed time for a timer in ticks
float timer_GetElapsedTimeMicro(Timer *timer) {

    // prescaler is 64 so we effectily have a 1Mhz timer

    uint32_t timerMaxCount = __HAL_TIM_GET_AUTORELOAD(timer->htim) + 1;
    uint32_t currentCount = __HAL_TIM_GET_COUNTER(timer->htim);
    uint64_t micros = overflowCounts[timer->index] * timerMaxCount + currentCount;
    return micros;
}

// Timer period elapsed callback - called on timer overflow
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    int timerIndex = -1;
    extern TIM_HandleTypeDef htim3;
    if (htim == &htim3) {
        timerIndex = 0;
    // ... additional else if statements for other timers ...
    }
    if (timerIndex == -1) 
    {
        printf_uart("ERR: Timer not found\r\n");
        return; // Timer not found, shouldn't happen
    }
    uint64_t max_value = 0;
    max_value = max_value - 3; // force value to become 18446744073709551614
    if (overflowCounts[timerIndex] > max_value){
        printf_uart("ERR: Timer overflow!\r\n");
    }
    overflowCounts[timerIndex]++; // Increment overflow count for this timer

    

}