#include "motor.h"

/* Private includes */
#include "log.h"
#include "timer.h"

/* Private variables */
static LOG_Module *internal_log_mod;

void MOTOR_Init()
{
  LOG_InitModule(internal_log_mod, "MOTOR", LOG_LEVEL_INFO);
}

#define CSLogic_Port 0x0
#define SER_Pin 0x0
#define SRCLK_Pin 0x0
#define RCLK_Pin 0x0
#define OE_Pin 0x0

void MOTOR_ConfigSPI(uint8_t drivers)
{
  for (int i = 0; i < 8; i++)
  {
    int cs = ((1 << i) & drivers) >> i;
    HAL_GPIO_WritePin(CSLogic_Port, SER_Pin, cs);
    HAL_GPIO_WritePin(CSLogic_Port, SRCLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CSLogic_Port, SRCLK_Pin, GPIO_PIN_RESET);
  }

  HAL_GPIO_WritePin(CSLogic_Port, RCLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CSLogic_Port, RCLK_Pin, GPIO_PIN_RESET);
  // Perhaps set OE on SPI begin?
  HAL_GPIO_WritePin(CSLogic_Port, OE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CSLogic_Port, OE_Pin, GPIO_PIN_RESET);
}

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
  changeDirection(motor, percent);
  runMotor(motor);

  // TODO: How to handle rounding errors, do they even matter?
  uint32_t pwm_speed = motor->htim->Init.Period * percent;
  LOG_DEBUG("pwm_speed: %d\r\n", pwm_speed);
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
  uint16_t count_amount = 100;
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
