#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"

typedef struct
{
  TIM_HandleTypeDef *encoder_htim;
  TIM_HandleTypeDef *pwm_htim;
  uint32_t channel;
  GPIO_TypeDef *breakPinPort;
  uint16_t breakPin;
  GPIO_TypeDef *reversePinPort;
  uint16_t reversePin;
  GPIO_TypeDef *encoderPinPort;
  uint16_t encoderPin;
  uint16_t reversing;
} MotorPWM;


/**
 * Initilaize motor
 */
void MOTOR_Init(TIM_HandleTypeDef* htim);

/**
 * Sets the break pin LOW
 */
void MOTOR_Stopbreak(MotorPWM *motor);

void MOTOR_PWMStart(MotorPWM *motor);

/**
 * Disables the PWM signal
 * @param motor Pointer to motor.
 */
void MOTOR_PWMStop(MotorPWM *motor);

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
