#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"


typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *reversePinPort;
  GPIO_TypeDef *breakPinPort;
  GPIO_TypeDef *readSpeedPinPort;
  uint16_t reversePin;
  uint16_t breakPin;
  uint16_t readSpeedPin;
  uint16_t reversing;
} MotorPWM;

/**
 * Start motor
 */
void MOTOR_Start(MotorPWM *motor);

/**
 * Stop motor
 */
void MOTOR_Stop(MotorPWM *motor);

/**
 * Set speed of motor in percent 0 - 100
 * Negative values are interpreted as reverse
 */
void MOTOR_SetSpeed(MotorPWM *motor, float percent);

/**
 * Sets the breaking pin
 */
void MOTOR_Break(MotorPWM *motor);

/**
 * ...
 */
float MOTOR_ReadSpeed(MotorPWM *motor);

#endif
