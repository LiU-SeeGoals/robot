#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"

typedef struct
{
  TIM_HandleTypeDef *encoder_htim;
  TIM_HandleTypeDef *pwm_htim;
  uint32_t channel;
  int ticks; // negative if overflow
  float speed; // negative if overflow
  int prev_tick; // negative if overflow
  GPIO_TypeDef *breakPinPort;
  uint16_t breakPin;
  GPIO_TypeDef *reversePinPort;
  uint16_t reversePin;
  GPIO_TypeDef *encoderPinPort;
  uint16_t encoderPin;
  uint16_t dir;
} MotorPWM;


/**
 * Initilaize motor
 */
void MOTOR_Init(TIM_HandleTypeDef* htim);

/**
 * Sets the break pin LOW
 */
void MOTOR_StopBreak(MotorPWM *motor);

void MOTOR_PWMStart(MotorPWM *motor);

/**
 * Disables the PWM signal
 * @param motor Pointer to motor.
 */
void MOTOR_PWMStop(MotorPWM *motor);

/**
 * Set speed of motor in percent 0 - 1
 * TODO: Negative values are interpreted as reverse
 */
void MOTOR_SendPWM(MotorPWM *motor, float pulse_width);

int printf_uart(const char *format, ...);

void MOTOR_SetSpeed(MotorPWM *motor, float speed, float* I_prev);

/**
 * Sets the breaking pin
 */
void MOTOR_Break(MotorPWM *motor);

int setDirection(MotorPWM *motor, float speed);

/**
 * ...
 */
float MOTOR_ReadSpeed(MotorPWM *motor);

#endif
