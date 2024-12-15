#include "nav.h"
#include "state_estimator.h"

/*
 * Private includes
 */
#include <stdlib.h>
#include "motor.h"
#include "log.h"
#include "arm_math.h"

#include <ringbuffer.h>

#define BUFFER_SIZE 64
RINGBUFFER_DEF(Command*, BUFFER_SIZE, Command_buf);
RINGBUFFER_IMPL(Command*, BUFFER_SIZE, Command_buf);

/*
 * Private variables
 */
static LOG_Module internal_log_mod;
static MotorPWM motors[4];
static float I_prevs[4] = {0.f, 0.f, 0.f, 0.f}; // PI control I-parts
float CLOCK_FREQ = 400000000;
float CONTROL_FREQ; // set in init
Command_buf queue;
static int queued = 0;

/* Private functions declarations */
void handle_command(Command* cmd);
void set_motors(float m1, float m2, float m3, float m4);

/*
 * Public function implementations
 */

void NAV_Init(TIM_HandleTypeDef* motor_tick_itr,
              TIM_HandleTypeDef* pwm_htim, 
              TIM_HandleTypeDef* pwm15_htim, 
              TIM_HandleTypeDef* encoder1_htim,
              TIM_HandleTypeDef* encoder2_htim,
              TIM_HandleTypeDef* encoder3_htim,
              TIM_HandleTypeDef* encoder4_htim) {

  LOG_InitModule(&internal_log_mod, "NAV", LOG_LEVEL_TRACE);
  HAL_TIM_Base_Start(pwm_htim);
  HAL_TIM_Base_Start(pwm15_htim);
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

#ifdef PCB_MOTOR
  motors[1].pwm_htim          = pwm15_htim;
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

#else
  motors[1].pwm_htim          = pwm_htim;
  motors[1].ticks             = 0;
  motors[1].speed             = 0.f;
  motors[1].prev_tick         = 0;
  motors[1].encoder_htim      = encoder2_htim;
  motors[1].channel           = TIM_CHANNEL_2;
  motors[1].breakPinPort      = OLD_MOTOR2_BREAK_GPIO_Port;
  motors[1].breakPin          = OLD_MOTOR2_BREAK_Pin;
  motors[1].reversePinPort    = OLD_MOTOR2_REVERSE_GPIO_Port;
  motors[1].reversePin        = OLD_MOTOR2_REVERSE_Pin;
  motors[1].dir               = 1;
#endif


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
}

void NAV_set_motor_ticks(){
  for (int i = 0; i < 4; i++){
    int ticks_before = motors[i].prev_tick;
    int new_ticks = motors[i].encoder_htim->Instance->CNT;
    motors[i].ticks = new_ticks - ticks_before;
    motors[i].prev_tick = new_ticks;
  }
  // Dont move this into the other for loop !!
  for (int i = 0; i < 4; i++){ // do for all motor
    MOTOR_SetSpeed(&motors[i], motors[i].speed, &I_prevs[i]);
  }

}

void steer(float vx,float vy, float w){

  /*float theta = 31.f * PI / 180.f;*/
  float theta = 31.f;
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
  motors[3].speed = v4;

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

void NAV_QueueCommandIRQ(Command* command) {
  if (!Command_buf_write(&queue, command)) {
    LOG_WARNING("Command buffer full\n\r");
  }
  ++queued;
}

void NAV_Stop() {
  MOTOR_PWMStop(&motors[0]);
  MOTOR_PWMStop(&motors[1]);
  MOTOR_PWMStop(&motors[2]);
  MOTOR_PWMStop(&motors[3]);
}

float speed = 0;

void command_move(Command *cmd){

  LOG_INFO("got nav command %d %d %d \r\n",cmd->kick_speed, cmd->command_id, cmd->direction->x, cmd->direction->y);
  if (cmd->command_id == ACTION_TYPE__STOP_ACTION){
    steer(0.f, 0.f, 0.f);
    return;
  }

  if (cmd->command_id == ACTION_TYPE__KICK_ACTION){
    speed = cmd->kick_speed;
    if (speed > 10){
      speed = 10;
    }
    return;
  }

  if (cmd->command_id == ACTION_TYPE__MOVE_ACTION){
    steer(100.f * speed * cmd->direction->x, 100.f * speed * cmd->direction->y, 0.f);
  }
}

void NAV_HandleCommands() {
  static int handled = 0;
  while (1) {
    Command *cmd;
    if (!Command_buf_read(&queue, &cmd)) {
      return;
    }
    ++handled;
    handle_command(cmd);
    protobuf_c_message_free_unpacked((ProtobufCMessage*) cmd, NULL);
  }
}

void NAV_TestMovement() {
  steer(0, 1, 0);
}

void NAV_StopMovement() {
  steer(0, 0, 0);
}


/*
 * Private function implementations
 */

void handle_command(Command* cmd){
  switch (cmd->command_id) {
    case ACTION_TYPE__STOP_ACTION:
      NAV_StopMovement();
      LOG_DEBUG("Stop\r\n");
      break;
    case ACTION_TYPE__MOVE_ACTION: {
      const int32_t speed = cmd->kick_speed;
      const int32_t x = cmd->direction->x;
      const int32_t y = cmd->direction->y;

      LOG_DEBUG("(x,y,speed): (%i,%i,%i)\r\n", x, y, speed);
      LOG_DEBUG("(x,y): (%f,%f)\r\n", 100.f*speed*x, 100.f*speed*y);
      // TODO: Should somehow know that we're in remote control mode
      if (0 <= speed && speed <= 10) {
        steer(100.f * speed * x, 100.f * speed * y, 0.f);
      }
      } break;
    case ACTION_TYPE__PING:
      break;
    case ACTION_TYPE__ROTATE_ACTION:
      break;
    case ACTION_TYPE__KICK_ACTION:
      break;
    default:
      LOG_WARNING("Not known command: %i\r\n", cmd->command_id);
      break;
  }
}

void NAV_TireTest() {
  LOG_INFO("Starting tire test...\r\n");

  LOG_INFO("First motor forward...\r\n");
  set_motors(1,0,0,0);
  HAL_Delay(2000);
  LOG_INFO("First motor backwards...\r\n");
  set_motors(-1,0,0,0);
  HAL_Delay(2000);

  LOG_INFO("Second motor forward...\r\n");
  set_motors(0,1,0,0);
  HAL_Delay(2000);
  LOG_INFO("Second motor backwards...\r\n");
  set_motors(0,-1,0,0);
  HAL_Delay(2000);

  LOG_INFO("Third motor forward...\r\n");
  set_motors(0,0,1,0);
  HAL_Delay(2000);
  LOG_INFO("Third motor backwards...\r\n");
  set_motors(0,0,-1,0);
  HAL_Delay(2000);

  LOG_INFO("Fourth motor forward...\r\n");
  set_motors(0,0,0,1);
  HAL_Delay(2000);
  LOG_INFO("Fourth motor Backwards...\r\n");
  set_motors(0,0,0,-1);
  HAL_Delay(2000);

  LOG_INFO("Finished tire test...\r\n");
}

void set_motors(float m1, float m2, float m3, float m4){
  motors[0].speed = m1 * 100.f;
  motors[1].speed = m2 * 100.f;
  motors[2].speed = m3 * 100.f;
  motors[3].speed = m4 * 100.f;
}

void NAV_StopDribbler(){
  HAL_GPIO_WritePin(DRIBBLER_GPIO_Port, DRIBBLER_Pin, GPIO_PIN_RESET);
}

void NAV_RunDribbler(){
  HAL_GPIO_WritePin(DRIBBLER_GPIO_Port, DRIBBLER_Pin, GPIO_PIN_SET);
}

void NAV_TestDribbler(){

  NAV_RunDribbler();
  HAL_Delay(2000);
  NAV_StopDribbler();

}
