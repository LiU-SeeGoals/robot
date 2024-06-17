#include "nav.h"

/*
 * Private includes
 */
#include <stdlib.h>
#include "motor.h"
#include "log.h"
#include <ringbuffer.h>

#define BUFFER_SIZE 64
RINGBUFFER_DEF(Command*, BUFFER_SIZE, Command_buf);
RINGBUFFER_IMPL(Command*, BUFFER_SIZE, Command_buf);

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


Command_buf queue;

static int queued = 0;

void NAV_QueueCommandIRQ(Command* command) {
  if (!Command_buf_write(&queue, command)) {
    LOG_WARNING("Command buffer full\n\r");
  }
  ++queued;
}

void NAV_HandleCommands() {
  static int handled = 0;
  while (1) {
    Command *cmd;
    if (!Command_buf_read(&queue, &cmd)) {
      return;
    }
    LOG_INFO("Handle command %d: %d, %d\n\r", cmd->command_id, handled, queued);
    ++handled;
    protobuf_c_message_free_unpacked(cmd, NULL);
  }
}
