#include "nav.h"

/*
 * Private includes
 */
#include <stdint.h>
#include "motor.h"
#include "log.h"

/*
 * Private variables
 */
static LOG_Module internal_log_mod;
static MotorPWM motors[4];


/*
 * Public function implementations
 */
void NAV_Init(TIM_HandleTypeDef* htim) {
  LOG_InitModule(&internal_log_mod, "NAV", LOG_LEVEL_DEBUG);

  motors[0].htim              = htim;
  motors[0].channel           = MOTOR1_TIM_CHANNEL;
  motors[0].breakPinPort      = MOTOR1_BREAK_GPIO_Port;
  motors[0].breakPin          = MOTOR1_BREAK_Pin;
  motors[0].reversePinPort    = MOTOR1_REVERSE_GPIO_Port;
  motors[0].reversePin        = MOTOR1_REVERSE_Pin;
  motors[0].encoderPinPort    = MOTOR1_ENCODER_GPIO_Port;
  motors[0].encoderPin        = MOTOR1_ENCODER_Pin;
  motors[0].reversing         = 0;

  motors[1].htim              = htim;
  motors[1].channel           = MOTOR2_TIM_CHANNEL;
  motors[1].breakPinPort      = MOTOR2_BREAK_GPIO_Port;
  motors[1].breakPin          = MOTOR2_BREAK_Pin;
  motors[1].reversePinPort    = MOTOR2_REVERSE_GPIO_Port;
  motors[1].reversePin        = MOTOR2_REVERSE_Pin;
  motors[1].encoderPinPort    = MOTOR2_ENCODER_GPIO_Port;
  motors[1].encoderPin        = MOTOR2_ENCODER_Pin;
  motors[1].reversing         = 0;

  motors[2].htim              = htim;
  motors[2].channel           = MOTOR3_TIM_CHANNEL;
  motors[2].breakPinPort      = MOTOR3_BREAK_GPIO_Port;
  motors[2].breakPin          = MOTOR3_BREAK_Pin;
  motors[2].reversePinPort    = MOTOR3_REVERSE_GPIO_Port;
  motors[2].reversePin        = MOTOR3_REVERSE_Pin;
  motors[2].encoderPinPort    = MOTOR3_ENCODER_GPIO_Port;
  motors[2].encoderPin        = MOTOR3_ENCODER_Pin;
  motors[2].reversing         = 0;

  motors[3].htim              = htim;
  motors[3].channel           = MOTOR4_TIM_CHANNEL;
  motors[3].breakPinPort      = MOTOR4_BREAK_GPIO_Port;
  motors[3].breakPin          = MOTOR4_BREAK_Pin;
  motors[3].reversePinPort    = MOTOR4_REVERSE_GPIO_Port;
  motors[3].reversePin        = MOTOR4_REVERSE_Pin;
  motors[3].encoderPinPort    = MOTOR4_ENCODER_GPIO_Port;
  motors[3].encoderPin        = MOTOR4_ENCODER_Pin;
  motors[3].reversing         = 0;

  MOTOR_PWMStart(&motors[0]);
  MOTOR_PWMStart(&motors[1]);
  MOTOR_PWMStart(&motors[2]);
  MOTOR_PWMStart(&motors[3]);

}

void test_motor(){

  float percent = 0.1;
  // MOTOR_PWMStart(&motors[3]);
  // MOTOR_SetSpeed(&motors[3], 1);
  int max_speed = 100;
  for (float i = 0; i < max_speed; i++){
    MOTOR_SetSpeed(&motors[3], i/100);
    HAL_Delay(20);
  }
  for (float i = max_speed; i > 0; i--){
    MOTOR_SetSpeed(&motors[3], i/100);
    HAL_Delay(20);
  }

}

void NAV_Direction(DIRECTION dir) {
  switch (dir) {
    case UP:
      MOTOR_PWMStart(&motors[0]);
      break;
    case DOWN:
      MOTOR_PWMStart(&motors[1]);
      break;
    case LEFT:
      MOTOR_PWMStart(&motors[2]);
      break;
    case RIGHT:
      MOTOR_PWMStart(&motors[3]);
      break;
  }
}

void NAV_Stop() {
  MOTOR_PWMStop(&motors[0]);
  MOTOR_PWMStop(&motors[1]);
  MOTOR_PWMStop(&motors[2]);
  MOTOR_PWMStop(&motors[3]);
}
