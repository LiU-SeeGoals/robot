#include "motor.h"

/* Private includes */
#include "log.h"
#include "timer.h"

/* Private variables */
static LOG_Module internal_log_mod;


void MOTOR_Init(TIM_HandleTypeDef* pwm_htim) {

  LOG_InitModule(&internal_log_mod, "MOTOR", LOG_LEVEL_TRACE);
  HAL_TIM_Base_Start(pwm_htim);

}

void MOTOR_PWMStop(MotorPWM *motor)
{
  // TODO: This might disable the timer for all channels, not sure.
  HAL_TIM_PWM_Stop(motor->pwm_htim, motor->channel);
}

void MOTOR_PWMStart(MotorPWM *motor)
{
  HAL_TIM_PWM_Start(motor->pwm_htim, motor->channel);
}

void MOTOR_Stopbreak(MotorPWM *motor)
{
  HAL_GPIO_WritePin(motor->breakPinPort, motor->breakPin, GPIO_PIN_RESET);
}

void MOTOR_Break(MotorPWM *motor)
{
  HAL_GPIO_WritePin(motor->breakPinPort, motor->breakPin, GPIO_PIN_SET);
}

/*
  DEPRECATED
  Reverses motor direction and makes sure that the motor is stopped before reversing
*/
void changeDirection(MotorPWM *motor, int percent)
{
  return; // will not work without all pins connected
  if ((motor->reversing && percent >= 0) || (!motor->reversing && percent <= 0))
  {
    while (MOTOR_ReadSpeed(motor) >= 0)
    {
      MOTOR_Break(motor);
    }
    if (percent < 0)
    {
      motor->reversing = 1;
    }
    else
    {
      motor->reversing = 0;
    }
    HAL_GPIO_WritePin(motor->breakPinPort, motor->reversePin, motor->reversing);
  }
}

  // TODO: How to not have globals? Will cause issues if function is not called
  // for some time
/*
  PI controls the motor to the given speed value in hall ticks / second
  Updates I_prev with the previous integrator value

*/

void MOTOR_SetSpeed(MotorPWM *motor, float speed, float* I_prev){
  // TODO: IMPORTANT add integrator windup protection since u is limimted 0 - 1

  // PI control loop with integrator windup protection
  float umin = 0;
  float umax = 1;
  float Ts = 0.01;
  float Ti = 0.05;
  float K = 0.001;
  float current_speed = MOTOR_ReadSpeed(motor);
  float error = speed - current_speed;
  float I = *I_prev + Ts / Ti * error;
  float v = K * (error + I);
  float u = 0;
  // integrator windup fix
  if (v < umin || v > umax){
    I = *I_prev;
    v = K * (error + I);
  }
  if (v > umax){
    u = umax;
  }
  else if (v < umin){
    u = umin;
  }
  else{
    u = v;
  }
  LOG_INFO("DATAu:%f;\r\n", u);
  MOTOR_SendPWM(motor, u);
  *I_prev = I;
  // TODO: How to not have globals? Will cause issues if function is not called
  // for some time
}

/*
  Send pwm with with pulse width of pulse_width * timer_period
  meaning pulse width is a float between 0 - 1.
*/

void MOTOR_SendPWM(MotorPWM *motor, float pulse_width)
{
  // TODO: How to handle changing directions?

  // Make sure we dont explode the timer limit
  if (pulse_width > 1){
    pulse_width = 1;
  }
  if (pulse_width < 0){
    pulse_width = 0;
  }

  float max_scale = 0.2; // Use a max scaling for the motor speed
  float scale = max_scale * pulse_width; // make max_scale largest scaling

  // TODO: How to handle rounding errors, do they even matter?
  int pwm_speed = motor->pwm_htim->Init.Period * scale;
  // LOG_INFO("pwm %d\r\n", pwm_speed);

  // pwm_speed = motor->pwm_htim->Init.Period * 0.2;

  __HAL_TIM_SET_COMPARE(motor->pwm_htim, motor->channel, pwm_speed);
}


void MOTOR_SetToTick(MotorPWM *motor, uint16_t tick)
{
  uint16_t ticks = 0;
  uint16_t ticks_before = motor->encoder_htim->Instance->CNT;
  while(ticks < tick)
  {
    uint16_t ticks_after = motor->encoder_htim->Instance->CNT;

    if (ticks_after != ticks_before){
      ticks += ticks_after - ticks_before;
      // LOG_DEBUG("tick: %d\r\n", ticks);
      ticks_before = ticks_after;
    }
  }
}


float MOTOR_ReadSpeed(MotorPWM *motor)
{
  LOG_DEBUG("ticks: %d\r\n", motor->ticks);

  float delay_ms = 100; // 100hz update -> 100ms update

  float speed_s = (motor->ticks) * 10;// 100ms update * 10 gives tick/second
  LOG_DEBUG("speed: %f\r\n", speed_s);

  // timer overflowed, so calculate again
  // could prob do some smart reverse calculations 
  // since we know it overflowed
  if (speed_s < 0) {
    speed_s = ( motor->ticks + 65536 - motor->prev_tick) * 10;// 10 gives tick/second
  }

  return speed_s;

}



// float MOTOR_ReadSpeed(MotorPWM *motor)
// {
//   // each pulse is one rotation of the motor
//   float radius = 0.1;                         // meters
//   float PI = 3.1415;                          // meters
//   float wheelCircumference = 2 * PI * radius; // meters

//   extern Timer timer3;
//   timer_start(&timer3);
//   // calcuate 100 up and downs
//   uint16_t count_amount = 3;
//   while (count_amount > 0)
//   {
//     if (HAL_GPIO_ReadPin(motor->encoderPinPort, motor->encoderPin))
//     {
//       count_amount--;
//       while (HAL_GPIO_ReadPin(motor->encoderPinPort, motor->encoderPin))
//       {
//         // wait for pin to go low
//       }
//     }
//   }
//   uint32_t time = timer_GetElapsedTimeMicro(&timer3);
//   timer_stop(&timer3);
//   float speed = wheelCircumference / (float)time;

//   return speed;
// }
