#include "nav.h"

/*
 * Private includes
 */
#include <stdlib.h>
#include "motor.h"
#include "log.h"
#include <stdatomic.h>

/*
 * Private variables
 */
static LOG_Module internal_log_mod;
static MotorPWM motors[4];


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
}

#define COMMAND_BUF_SIZE 64
atomic_uint command_val;
Command * volatile commands[COMMAND_BUF_SIZE];

void NAV_QueueCommandIRQ(Command* command) {
  unsigned command_count = command_val >> 4;
  unsigned command_ix = command_val & 0x0f;
  if (command_count == COMMAND_BUF_SIZE) {
    LOG_WARNING("Command buffer full\n\r");
    return;
  }
  unsigned end = (command_ix + command_count) % COMMAND_BUF_SIZE;
  command_val += (1 << 4);
  commands[end] = command;
}


void NAV_HandleCommands() {
  static int handled = 0;
  while (1) {
    unsigned command = command_val;
    unsigned command_count = command >> 4;
    unsigned command_ix = command & 0x0f;
    if (command_count == 0) {
      return;
    }
    Command* to_handle = commands[command_ix];
    LOG_INFO("Handle command %d: %d\n\r", to_handle->command_id, handled);
    ++handled;
    command_count -= 1;
    command_ix += 1;
    command_val = command_count << 4 & command_ix;
    free(to_handle);
  }
}
