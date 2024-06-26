#ifndef NAV_H
#define NAV_H

/*
 * Public includes
 */
#include "main.h"
#include <robot_action/robot_action.pb-c.h>

typedef enum {
  UP,
  DOWN,
  LEFT,
  RIGHT
} DIRECTION;

/**
 * Initialises navigation system by creating motor interfaces.
 *
 * @param htim Timer handle for motors.
 */
void NAV_Init(TIM_HandleTypeDef* htim);

/**
* Queues a command from the basestation.
* @param Command The command.
*/
void NAV_QueueCommandIRQ(Command* command);
/**
 * Handle all received commands.
 */
void NAV_HandleCommands();

#endif /* NAV_H */
