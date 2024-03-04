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

typedef struct Vector3D
{
  float x;
  float y;
  float w;
} Vector3D;

/**
 * Initialises navigation system by creating motor interfaces.
 *
 * @param htim Timer handle for motors.
 */
void NAV_Init(TIM_HandleTypeDef *htim);

/**
 *
 */
void NAV_Direction(DIRECTION dir);

void NAV_Stop();

void NAV_Move(Vector3D pos, Vector3D dest);

void NAV_Steer(float vx, float vy, float w);

Vector3D NAV_CalculateSpeed();

#endif /* NAV_H */
