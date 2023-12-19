#include "motor.h"

#include "timer.h"

void MOTOR_Start(MotorPWM *motor)
{
  HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

void MOTOR_Stop(MotorPWM *motor)
{
  HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}

void runMotor(MotorPWM *motor)
{
  HAL_GPIO_WritePin(motor->breakPinPort, motor->breakPin, GPIO_PIN_RESET);
}

void MOTOR_Break(MotorPWM *motor)
{
  HAL_GPIO_WritePin(motor->breakPinPort, motor->breakPin, GPIO_PIN_SET);
}

/*
  Reverses motor direction and makes sure that the motor is stopped before reversing
*/
void changeDirection(MotorPWM *motor, int percent)
{
  return; // will not work without all pins connected
  if ((motor->reversing && percent >= 0) || (!motor->reversing && percent <= 0))
  {
    while(MOTOR_ReadSpeed(motor) >= 0){
      MOTOR_Break(motor);
    }
    if (percent < 0)
    {
      motor->reversing = 1;
    }
    else
    {
      motor->reversing = 0;
    }
    HAL_GPIO_WritePin(motor->breakPinPort, motor->reversePin, motor->reversing);
  }
}

void MOTOR_SetSpeed(MotorPWM *motor, float percent)
{
  changeDirection(motor, percent);
  runMotor(motor);

  // TODO: How to handle rounding errors, do they even matter?
  uint32_t pwm_speed = motor->htim->Init.Period * percent;
  printf("[MOTOR] pwm_speed: %d\n", pwm_speed);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_speed);
}

float MOTOR_ReadSpeed(MotorPWM *motor)
{
  // each pulse is one rotation of the motor
  float radius = 0.1; // meters
  float PI = 3.1415; // meters
  float wheelCircumference = 2 * PI * radius; // meters

  extern Timer timer3;
  timer_start(&timer3);
  // calcuate 100 up and downs
  uint16_t count_amount = 100;
  while (count_amount > 0)
  {
    if (HAL_GPIO_ReadPin(motor->readSpeedPinPort, motor->readSpeedPin))
    {
      count_amount--;
      while (HAL_GPIO_ReadPin(motor->readSpeedPinPort, motor->readSpeedPin))
      {
        // wait for pin to go low
      }
    }
  }
  uint32_t time = timer_GetElapsedTimeMicro(&timer3);
  timer_stop(&timer3);
  float speed = wheelCircumference / (float) time;

  return speed;
}
