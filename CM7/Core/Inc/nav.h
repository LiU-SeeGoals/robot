#ifndef NAV_H
#define NAV_H

/*
 * Public includes
 */
#include "main.h"

typedef enum {
  UP,
  DOWN,
  LEFT,
  RIGHT
} DIRECTION;

/**
 * Initialises navigation system by creating motor interfaces.
 *
 * @param htim Timer handle for motors.
 */

void NAV_Init(TIM_HandleTypeDef* pwm_htim, 
              TIM_HandleTypeDef* encoder1_htim,
              TIM_HandleTypeDef* encoder2_htim,
              TIM_HandleTypeDef* encoder3_htim,
              TIM_HandleTypeDef* encoder4_htim);

/**
 *
 */
void NAV_Direction(DIRECTION dir);

void NAV_Stop();


#endif /* NAV_H */
