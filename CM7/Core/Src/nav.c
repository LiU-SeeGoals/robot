#include "nav.h"

/*
 * Private includes
 */
#include <stdint.h>
#include "motor.h"
#include "log.h"
#include "timer.h"

/*
 * Private variables
 */
static LOG_Module internal_log_mod;
static MotorPWM motors[4];
static Timer timer;

/*
 * Public function implementations
 */
void NAV_Init(TIM_HandleTypeDef* htim) {
  LOG_InitModule(&internal_log_mod, "NAV", LOG_LEVEL_INFO);

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

  timer = (Timer){.htim = htim, .index = 0};
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

static Vector3D vel = {0};

void NAV_Steer(float vx, float vy, float w)
{
  vel = (Vector3D){.x = vx, .y = vy, .w = w};
}

Vector3D NAV_CalculateSpeed()
{
  return vel;
}

void NAV_Move(Vector3D pos, Vector3D dest)
{
  float elapsed_seconds = 0;
  float pos_x = pos.x, pos_y = pos.y, pos_w = pos.w;
  timer_start(&timer);
  while (fabsf(pos_w - dest.w) > 0.1 || fabsf(pos_x - dest.x) > 0.1 || fabsf(pos_y - dest.y) > 0.1)
  {
    elapsed_seconds = timer_GetElapsedTimeMicro(&timer) / MICROS_IN_SEC;
    timer_start(&timer);
    // NOTE: pos and dest may be global, NAV_Steer&CalculateSpeed local.
    // TODO: Implement conversion from global to local, nicer reg. etc.
    LOG_INFO("pos.x=%f pos.y=%f pos.w=%f\r\n", pos_x, pos_y, pos_w); //, (int)((pos.w - (int)pos.w) * 1000000));
    LOG_INFO("elapsed_seconds=%f\r\n", elapsed_seconds);
    float rotate_diff = dest.w - pos_w;
    float x_diff = dest.x - pos_x;
    float y_diff = dest.y - pos_y;
    Vector3D vel = NAV_CalculateSpeed();
    NAV_Steer(x_diff / 3.0, y_diff / 2.0, rotate_diff);
    pos_w += vel.w * elapsed_seconds;
    pos_x += vel.x * elapsed_seconds;
    pos_y += vel.y * elapsed_seconds;
  }

  timer_stop(&timer);
}