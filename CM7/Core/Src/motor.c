#include "motor.h"

/* Private function declarations */
void runMotor(MotorPWM *motor);
void changeDirection(MotorPWM *motor, int percent);


/*
 * Public functions implementations
 */

void MOTOR_StartMotor(MotorPWM *motor){
  HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

void MOTOR_BreakMotor(MotorPWM *motor)
{
  // Sets the breaking pin
  HAL_GPIO_WritePin(&motor->breakPinPort, motor->breakPin, GPIO_PIN_SET);
}

void MOTOR_SetSpeed(MotorPWM *motor, float percent)
{
  // Set speed of motor in percent 0 - 100
  // Negative values are interpreted as reverse
  changeDirection(motor, percent);
  runMotor(motor);

  // TODO: How to handle rounding errors, do they even matter?
  uint32_t pwm_speed = motor->htim->Init.Period * percent;
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_speed);
}

int MOTOR_ReadSpeed(MotorPWM *motor)
{
  return HAL_GPIO_ReadPin(&motor->readSpeedPinPort, motor->readSpeedPin);
}


/*
 * Private function implementations
 */

void runMotor(MotorPWM *motor)
{
  // Resets the breaking pin
  HAL_GPIO_WritePin(&motor->breakPinPort, motor->breakPin, GPIO_PIN_RESET);
}

void changeDirection(MotorPWM *motor, int percent)
{
  // Reverses motor direction and makes sure that the motor is stopped before reversing
  if ((motor->reversing && percent >= 0) || (!motor->reversing && percent <= 0))
  {
    while(MOTOR_ReadSpeed(motor) >= 0){
      MOTOR_BreakMotor(motor);
    }
    if (percent < 0)
    {
      motor->reversing = 1;
    }
    else
    {
      motor->reversing = 0;
    }
    HAL_GPIO_WritePin(&motor->breakPinPort, motor->reversePin, motor->reversing);
  }
}
