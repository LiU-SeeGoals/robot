#ifndef NAV_H
#define NAV_H

/*
 * Public includes
 */
#include <stdint.h>
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
s*/
void NAV_Init(TIM_HandleTypeDef* htim);

/**
 *
 */
void NAV_Direction(DIRECTION dir);

void NAV_Stop();


#endif /* NAV_H */
