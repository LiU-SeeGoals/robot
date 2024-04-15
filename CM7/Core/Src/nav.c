#include "nav.h"

/*
 * Private includes
 */
#include <stdint.h>
#include "motor.h"
#include "log.h"
#include "arm_math.h"


/*
 * Private variables
 */
static LOG_Module internal_log_mod;
static MotorPWM motors[4];
static float I_prevs[4] = {0.f, 0.f, 0.f, 0.f}; // PI control I-parts
float CLOCK_FREQ = 400000000;
float CONTROL_FREQ; // set in init


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
  motors[0].speed             = 0.f;
  motors[0].prev_tick         = 0;
  motors[0].encoder_htim      = encoder1_htim;
  motors[0].channel           = MOTOR1_TIM_CHANNEL;
  motors[0].breakPinPort      = MOTOR1_BREAK_GPIO_Port;
  motors[0].breakPin          = MOTOR1_BREAK_Pin;
  motors[0].reversePinPort    = MOTOR1_REVERSE_GPIO_Port;
  motors[0].reversePin        = MOTOR1_REVERSE_Pin;
  motors[0].encoderPinPort    = MOTOR1_ENCODER_GPIO_Port;
  motors[0].encoderPin        = MOTOR1_ENCODER_Pin;
  motors[0].dir               = 1;

  motors[1].pwm_htim          = pwm_htim;
  motors[1].ticks             = 0;
  motors[1].speed             = 0.f;
  motors[1].prev_tick         = 0;
  motors[1].encoder_htim      = encoder2_htim;
  motors[1].channel           = MOTOR2_TIM_CHANNEL;
  motors[1].breakPinPort      = MOTOR2_BREAK_GPIO_Port;
  motors[1].breakPin          = MOTOR2_BREAK_Pin;
  motors[1].reversePinPort    = MOTOR2_REVERSE_GPIO_Port;
  motors[1].reversePin        = MOTOR2_REVERSE_Pin;
  motors[1].dir               = 1;

  motors[2].pwm_htim          = pwm_htim;
  motors[2].ticks             = 0;
  motors[2].speed             = 0.f;
  motors[2].prev_tick         = 0;
  motors[2].encoder_htim      = encoder3_htim;
  motors[2].channel           = MOTOR3_TIM_CHANNEL;
  motors[2].breakPinPort      = MOTOR3_BREAK_GPIO_Port;
  motors[2].breakPin          = MOTOR3_BREAK_Pin;
  motors[2].reversePinPort    = MOTOR3_REVERSE_GPIO_Port;
  motors[2].reversePin        = MOTOR3_REVERSE_Pin;
  motors[2].dir               = 1;

  motors[3].pwm_htim          = pwm_htim;
  motors[3].ticks             = 0;
  motors[3].speed             = 0.f;
  motors[3].prev_tick         = 0;
  motors[3].encoder_htim      = encoder4_htim;
  motors[3].channel           = MOTOR4_TIM_CHANNEL;
  motors[3].breakPinPort      = MOTOR4_BREAK_GPIO_Port;
  motors[3].breakPin          = MOTOR4_BREAK_Pin;
  motors[3].reversePinPort    = MOTOR4_REVERSE_GPIO_Port;
  motors[3].reversePin        = MOTOR4_REVERSE_Pin;
  motors[3].dir               = 1;

  MOTOR_PWMStart(&motors[0]);
  MOTOR_PWMStart(&motors[1]);
  MOTOR_PWMStart(&motors[2]);
  MOTOR_PWMStart(&motors[3]);

  float control_clock_prescaler = motor_tick_itr->Init.Prescaler + 1; 
  float control_clock_period = motor_tick_itr->Init.Period + 1;
  CONTROL_FREQ = CLOCK_FREQ / (control_clock_prescaler * control_clock_period);
  HAL_TIM_Base_Start_IT(motor_tick_itr);
  // LOG_INFO("control time %f \r\n", CONTROL_FREQ);

}


// float I_prev = 0.f;
void NAV_set_motor_ticks(){
  for (int i = 0; i < 4; i++){
    int ticks_before = motors[i].prev_tick;
    int new_ticks = motors[i].encoder_htim->Instance->CNT;
    motors[i].ticks = new_ticks - ticks_before;
    motors[i].prev_tick = new_ticks;
  }
  for (int i = 0; i < 4; i++){ // do for all motor
    MOTOR_SetSpeed(&motors[i], motors[i].speed, &I_prevs[i]);
  }

}

void steer(float vx,float vy, float w){

  // float theta = 31.f * PI / 180.f;
  float theta = 45.f;
  float psi = 45.f;
  float r = 1.f;
  float th_sin, th_cos;
  float psi_sin, psi_cos;

  arm_sin_cos_f32(theta, &th_sin, &th_cos);
  arm_sin_cos_f32(psi, &psi_sin, &psi_cos);

  float v1 = th_sin * vx +  th_cos * vy + -r * w;
  float v2 = th_sin * vx + -th_cos * vy + -r * w;
  float v3 = -psi_sin  * vx +  -psi_cos *  vy+  -r * w;
  float v4 = -psi_sin  * vx +  psi_cos *  vy + -r * w;
  // float v4 = -th_cos;
  // v1 = sin(vx * theta * PI / 180.f);
  motors[0].speed = v1;
  motors[1].speed = v2;
  motors[2].speed = v3;
  // motors[3].speed = v41 + v42;
  motors[3].speed = v4;
  LOG_INFO("v1 %f,v2 %f,v3 %f,v4 %f  \r\n", v1,v2,v3,v4);
  // LOG_INFO("v4 %f \r\n", v4);
  // LOG_INFO("hej \r\n");
  // HAL_Delay(10);

}

void test_motor() {
  // MOTOR_SetSpeed(&motors[3], -70.0, &I_prev);
  steer(-1.f * 100.f, 1.f * 100.f, 0.f);
  // motors[0].speed = 0;
  // motors[1].speed = 4.f * 100.f;
  // motors[2].speed = 0;
  // motors[3].speed = 12.f * 100.f;


  // float speed = MOTOR_ReadSpeed(&motors[3]);
  // LOG_INFO("control freq %f\r\n", CONTROL_FREQ);
  // HAL_Delay(5000);
  // HAL_GPIO_WritePin(motors[3].reversePinPort, motors[3].reversePin, GPIO_PIN_SET);
  // MOTOR_SendPWM(&motors[3], 0.5);
  // MOTOR_SendPWM(&motors[3], 0.15);
  // HAL_Delay(2000);

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
