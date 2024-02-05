#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *breakPinPort;
  uint16_t breakPin;
  GPIO_TypeDef *reversePinPort;
  uint16_t reversePin;
  GPIO_TypeDef *encoderPinPort;
  uint16_t encoderPin;
  uint16_t reversing;
} MotorPWM;
uint8_t DRIVER_1 = 1 << 0;
uint8_t DRIVER_2 = 1 << 1;
uint8_t DRIVER_3 = 1 << 2;
uint8_t DRIVER_4 = 1 << 3;
uint8_t DRIVER_5 = 1 << 4;


void MOTOR_Init();

/**
 * Start motor
 */
void MOTOR_Start(MotorPWM *motor);

/**
 * Stop motor
 * @param motor Pointer to motor.
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

/**
 * Configures which motor drivers to communicate with when using SPI.
 *
 * @param drivers Which drivers to communicate with, encoded in an
 * 8 bit integer. E.g. setting LSB means selecting driver 1.
 */
void MOTOR_ConfigSPI(uint8_t drivers);

#endif
