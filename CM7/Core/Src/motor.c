#include "motor.h"

/* Private includes */
#include "log.h"
#include "timer.h"

/* Private variables */
static LOG_Module internal_log_mod;
extern float CONTROL_FREQ;

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
  Reverses motor direction and makes sure that the motor is stopped before reversing
*/
int setDirection(MotorPWM *motor, float speed)
{

  // If going backward and speed is positive, change direction
  if (motor->dir == 0 && speed > 0)
  {
    // if we are to change direction but motor is not stopped, return;
    if (!(MOTOR_ReadSpeed(motor) == 0))
    {
      return HAL_BUSY;
    }
    motor->dir = 1;
    LOG_DEBUG("going backchinging dir: %d\r\n", motor->dir);
    HAL_GPIO_WritePin(motor->reversePinPort, motor->reversePin, GPIO_PIN_RESET);
  }
  // If going forward and speed is negative, change direction
  if (motor->dir == 1 && speed < 0)
  {
    // if we are to change direction but motor is not stopped, return;
    if (!(MOTOR_ReadSpeed(motor) == 0))
    {
      return HAL_BUSY;
    }
    motor->dir = 0;
    /*LOG_DEBUG("going for dir: %d\r\n", motor->dir);*/
    HAL_GPIO_WritePin(motor->reversePinPort, motor->reversePin, GPIO_PIN_SET);
  }

  return HAL_OK;
}

  // TODO: How to not have globals? Will cause issues if function is not called
  // for some time
/*
  PI controls the motor to the given speed value in hall ticks / second
  Updates I_prev with the previous integrator value

*/
void MOTOR_SetSpeed(MotorPWM *motor, float speed, float* I_prev){

  if (setDirection(motor, speed) == HAL_BUSY) {
    MOTOR_SendPWM(motor, 0);
  }
  if (speed < 0) {
    speed = -speed;
  }
  // PI control loop with integrator windup protection
  
  float umin = 0;
  float umax = 1;
  float Ts = 1.f / CONTROL_FREQ;
  float Ti = 0.02;
  float K = 0.00015;
  float current_speed = (float) MOTOR_ReadSpeed(motor);
  float error = speed - current_speed;
  float I = *I_prev + Ts / Ti * error;
  float v = K * (error + I);
  float u = 0;
  // integrator windup fix
  if (v < umin || v > umax){
    I = *I_prev;
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
  /*LOG_INFO("DATAu:%f;\r\n", u);*/
  // HAL_Delay(1);
  MOTOR_SendPWM(motor, u);
  *I_prev = I;
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

  float max_scale = 0.5; // Use a max scaling for the motor speed
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
  // LOG_DEBUG("ticks: %d\r\n", motor->ticks);


  float speed_s = (float)(motor->ticks) *  CONTROL_FREQ; // 10ms update * 10 gives tick/second
  // LOG_DEBUG("speed: %f\r\n", speed_s);

  // timer overflowed, so calculate again
  // could prob do some smart reverse calculations 
  // since we know it overflowed
  // TODO: Monka race condition with timer
  // if (speed_s < 0) {
  //   speed_s = (float)( motor->ticks + 65536 - motor->prev_tick) * (float) CONTROL_FREQ;// 10 gives tick/second
  // }

  return speed_s;

}
