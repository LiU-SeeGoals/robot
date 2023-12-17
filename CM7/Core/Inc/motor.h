#ifndef __MOTOR_H
#define __MOTOR_H


/* Public includes */
#include "main.h"


/* Public structs */

/**
 * Describe this
 */
typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  uint16_t reversePin;
  GPIO_TypeDef reversePinPort;
  uint16_t breakPin;
  GPIO_TypeDef breakPinPort;
  uint16_t readSpeedPin;
  GPIO_TypeDef readSpeedPinPort;
  uint16_t reversing;
} MotorPWM;


/* Public function declarations */

/**
 * Describe this
 *
 * @param motor Describe this
 */
void MOTOR_StartMotor(MotorPWM *motor);

/**
 * Describe this
 *
 * @param motor Describe this
 * @param percent Describe this
 */
void MOTOR_SetSpeed(MotorPWM *motor, float percent);

/**
 * Describe this
 *
 * @param motor Describe this
 */
void MOTOR_BreakMotor(MotorPWM *motor);

/**
 * Describe this
 *
 * @param motor Describe this
 */
int MOTOR_ReadSpeed(MotorPWM *motor);

#endif /* __MOTOR_H */
