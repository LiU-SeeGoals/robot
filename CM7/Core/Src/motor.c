#include "motor.h"

/* Private includes */
#include "log.h"
#include "timer.h"

/* Private variables */
static LOG_Module internal_log_mod;


void MOTOR_Init(TIM_HandleTypeDef* htim)
{  
  LOG_InitModule(&internal_log_mod, "MOTOR", LOG_LEVEL_INFO);
  HAL_TIM_Base_Start(htim);
}

void MOTOR_PWMStop(MotorPWM *motor)
{
  // TODO: This might disable the timer for all channels, not sure.
  HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}

void MOTOR_PWMStart(MotorPWM *motor)
{
  HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

void MOTOR_Stopbreak(MotorPWM *motor)
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
    while (MOTOR_ReadSpeed(motor) >= 0)
    {
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
  // TODO: How to handle changing directions?

  // Make sure we dont explode the timer limit
  if (percent > 1){
    percent = 1;
  }
  if (percent < 0){
    percent = 0;
  }

  float max_scale = 0.5;
  float scale = max_scale * percent; // make max_scale largest

  // TODO: How to handle rounding errors, do they even matter?
  int pwm_speed = motor->htim->Init.Period * scale;
  LOG_INFO("pwm %d\r\n", pwm_speed);

  // pwm_speed = motor->htim->Init.Period * 0.2;

  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_speed);
}

float MOTOR_ReadSpeed(MotorPWM *motor)
{
  // each pulse is one rotation of the motor
  float radius = 0.1;                         // meters
  float PI = 3.1415;                          // meters
  float wheelCircumference = 2 * PI * radius; // meters

  extern Timer timer3;
  timer_start(&timer3);
  // calcuate 100 up and downs
  uint16_t count_amount = 3;
  while (count_amount > 0)
  {
    if (HAL_GPIO_ReadPin(motor->encoderPinPort, motor->encoderPin))
    {
      count_amount--;
      while (HAL_GPIO_ReadPin(motor->encoderPinPort, motor->encoderPin))
      {
        // wait for pin to go low
      }
    }
  }
  uint32_t time = timer_GetElapsedTimeMicro(&timer3);
  timer_stop(&timer3);
  float speed = wheelCircumference / (float)time;

  return speed;
}
