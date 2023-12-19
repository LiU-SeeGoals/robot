#include "nav.h"

/*
 * Private includes
 */
#include "motor.h"

/*
 * Private variables
 */
MotorPWM motors[4];


/*
 * Public function implementations
 */

void NAV_Init(TIM_HandleTypeDef* htim) {
  motors[0].htim              = htim;
  motors[0].channel           = MOTOR1_TIM_CHANNEL;
  motors[0].breakPinPort      = MOTOR1_BREAK_GPIO_Port;
  motors[0].breakPin          = MOTOR1_BREAK_Pin;
  motors[0].reversePinPort    = MOTOR1_REVERSE_GPIO_Port;
  motors[0].reversePin        = MOTOR1_REVERSE_Pin;
  motors[0].readSpeedPinPort  = MOTOR1_ENCODER_GPIO_Port;
  motors[0].readSpeedPin      = MOTOR1_ENCODER_Pin;
  motors[0].reversing         = 0;

  motors[1].htim              = htim;
  motors[1].channel           = MOTOR2_TIM_CHANNEL;
  motors[1].breakPinPort      = MOTOR2_BREAK_GPIO_Port;
  motors[1].breakPin          = MOTOR2_BREAK_Pin;
  motors[1].reversePinPort    = MOTOR2_REVERSE_GPIO_Port;
  motors[1].reversePin        = MOTOR2_REVERSE_Pin;
  motors[1].readSpeedPinPort  = MOTOR2_ENCODER_GPIO_Port;
  motors[1].readSpeedPin      = MOTOR2_ENCODER_Pin;
  motors[1].reversing         = 0;

  motors[2].htim              = htim;
  motors[2].channel           = MOTOR3_TIM_CHANNEL;
  motors[2].breakPinPort      = MOTOR3_BREAK_GPIO_Port;
  motors[2].breakPin          = MOTOR3_BREAK_Pin;
  motors[2].reversePinPort    = MOTOR3_REVERSE_GPIO_Port;
  motors[2].reversePin        = MOTOR3_REVERSE_Pin;
  motors[2].readSpeedPinPort  = MOTOR3_ENCODER_GPIO_Port;
  motors[2].readSpeedPin      = MOTOR3_ENCODER_Pin;
  motors[2].reversing         = 0;

  motors[3].htim              = htim;
  motors[3].channel           = MOTOR4_TIM_CHANNEL;
  motors[3].breakPinPort      = MOTOR4_BREAK_GPIO_Port;
  motors[3].breakPin          = MOTOR4_BREAK_Pin;
  motors[3].reversePinPort    = MOTOR4_REVERSE_GPIO_Port;
  motors[3].reversePin        = MOTOR4_REVERSE_Pin;
  motors[3].readSpeedPinPort  = MOTOR4_ENCODER_GPIO_Port;
  motors[3].readSpeedPin      = MOTOR4_ENCODER_Pin;
  motors[3].reversing         = 0;
}

void NAV_Direction(DIRECTION dir) {
  switch (dir) {
    case UP:
      MOTOR_Start(&motors[0]);
      break;
    case DOWN:
      MOTOR_Start(&motors[1]);
      break;
    case LEFT:
      MOTOR_Start(&motors[2]);
      break;
    case RIGHT:
      MOTOR_Start(&motors[3]);
      break;
  }
}

void NAV_Stop() {
  MOTOR_Stop(&motors[0]);
  MOTOR_Stop(&motors[1]);
  MOTOR_Stop(&motors[2]);
  MOTOR_Stop(&motors[3]);
}
