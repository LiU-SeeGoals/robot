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

void NAV_Init(TIM_HandleTypeDef* motor_tick_itr,
              TIM_HandleTypeDef* pwm_htim, 
              TIM_HandleTypeDef* encoder1_htim,
              TIM_HandleTypeDef* encoder2_htim,
              TIM_HandleTypeDef* encoder3_htim,
              TIM_HandleTypeDef* encoder4_htim) {

  LOG_InitModule(&internal_log_mod, "NAV", LOG_LEVEL_TRACE);
  HAL_TIM_Base_Start(pwm_htim);
  HAL_TIM_Base_Start(encoder1_htim);
  HAL_TIM_Base_Start(encoder2_htim);
  HAL_TIM_Base_Start(encoder3_htim);
  HAL_TIM_Base_Start(encoder4_htim);

  motors[0].pwm_htim          = pwm_htim;
  motors[0].ticks             = 0;
  motors[0].prev_tick         = 0;
  motors[0].encoder_htim      = encoder1_htim;
  motors[0].channel           = MOTOR1_TIM_CHANNEL;
  motors[0].breakPinPort      = MOTOR1_BREAK_GPIO_Port;
  motors[0].breakPin          = MOTOR1_BREAK_Pin;
  motors[0].reversePinPort    = MOTOR1_REVERSE_GPIO_Port;
  motors[0].reversePin        = MOTOR1_REVERSE_Pin;
  motors[0].encoderPinPort    = MOTOR1_ENCODER_GPIO_Port;
  motors[0].encoderPin        = MOTOR1_ENCODER_Pin;
  motors[0].reversing         = 0;

  motors[1].pwm_htim          = pwm_htim;
  motors[1].ticks             = 0;
  motors[1].prev_tick         = 0;
  motors[1].encoder_htim      = encoder2_htim;
  motors[1].channel           = MOTOR2_TIM_CHANNEL;
  motors[1].breakPinPort      = MOTOR2_BREAK_GPIO_Port;
  motors[1].breakPin          = MOTOR2_BREAK_Pin;
  motors[1].reversePinPort    = MOTOR2_REVERSE_GPIO_Port;
  motors[1].reversePin        = MOTOR2_REVERSE_Pin;
  motors[1].reversing         = 0;

  motors[2].pwm_htim          = pwm_htim;
  motors[2].ticks             = 0;
  motors[2].prev_tick         = 0;
  motors[2].encoder_htim      = encoder3_htim;
  motors[2].channel           = MOTOR3_TIM_CHANNEL;
  motors[2].breakPinPort      = MOTOR3_BREAK_GPIO_Port;
  motors[2].breakPin          = MOTOR3_BREAK_Pin;
  motors[2].reversePinPort    = MOTOR3_REVERSE_GPIO_Port;
  motors[2].reversePin        = MOTOR3_REVERSE_Pin;
  motors[2].reversing         = 0;

  motors[3].pwm_htim          = pwm_htim;
  motors[3].ticks             = 0;
  motors[3].prev_tick         = 0;
  motors[3].encoder_htim      = encoder4_htim;
  motors[3].channel           = MOTOR4_TIM_CHANNEL;
  motors[3].breakPinPort      = MOTOR4_BREAK_GPIO_Port;
  motors[3].breakPin          = MOTOR4_BREAK_Pin;
  motors[3].reversePinPort    = MOTOR4_REVERSE_GPIO_Port;
  motors[3].reversePin        = MOTOR4_REVERSE_Pin;
  motors[3].reversing         = 0;

  MOTOR_PWMStart(&motors[0]);
  MOTOR_PWMStart(&motors[1]);
  MOTOR_PWMStart(&motors[2]);
  MOTOR_PWMStart(&motors[3]);

  HAL_TIM_Base_Start_IT(motor_tick_itr);
}


void NAV_set_motor_ticks(){
  for (int i = 0; i < 4; i++){
    int ticks_before = motors[i].prev_tick;
    int new_ticks = motors[i].encoder_htim->Instance->CNT;
    motors[i].ticks = new_ticks - ticks_before;
    motors[i].prev_tick = new_ticks;
  }
}

void test_motor() {
  MOTOR_SetSpeed(&motors[3], 100);
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
