#ifndef NAV_H
#define NAV_H

/*
 * Public includes
 */
#include "main.h"
#include <robot_action/robot_action.pb-c.h>

// Which positions robot should go to, sent by AI dudes

typedef struct
{
  float x;
  float y;
  float w;
} robot_nav_command;

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
void NAV_Init(TIM_HandleTypeDef* motor_tick_itr,
              TIM_HandleTypeDef* pwm_htim, 
              TIM_HandleTypeDef* pwm15_htim, 
              TIM_HandleTypeDef* encoder1_htim,
              TIM_HandleTypeDef* encoder2_htim,
              TIM_HandleTypeDef* encoder3_htim,
              TIM_HandleTypeDef* encoder4_htim);

void NAV_set_motor_ticks();

void steer(float vx,float vy, float w);
/**
* Queues a command from the basestation.
* @param Command The command.
*/
void NAV_QueueCommandIRQ(Command* command);

/**
 * Handle all received commands.
 */
void NAV_HandleCommands();

/**
 * Stops all movement
 */
void NAV_StopMovement();

void NAV_TestMovement();

void NAV_StopDribbler();

void NAV_RunDribbler();

void NAV_TestDribbler();

void NAV_log_speed();

void NAV_TireTest();

robot_nav_command NAV_GetNavCommand();

void NAV_GoToAction(Command* command);

#endif /* NAV_H */

