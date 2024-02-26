#ifndef NAV_H
#define NAV_H

/*
 * Public includes
 */
#include <robot_action.pb.h>
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
void NAV_Init(TIM_HandleTypeDef* htim);

/**
 *
 */
void NAV_Direction(DIRECTION dir);

void NAV_Stop();

void NAV_Move(_action_Vector3D pos, _action_Vector3D dest);

#endif /* NAV_H */
