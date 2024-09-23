#include "../Inc/HandmadeMath.h"
#include "math.h"
#include "../Inc/state_estimator.h"

/*extern Vec2 statex;*/
/*extern Vec2 statey;*/
/*extern Vec2 statew;*/
// Each state has an integration part in the pid controller
Vec2 statex_I;
Vec2 statey_I;
Vec2 statew_I;

/*extern Mat2 Px;*/
/*extern Mat2 Py;*/
/*extern Mat2 Pw;*/

float angle_error(float angle, float desired){

  // TODO make sure returned sign is correct for the desired direction
  float left_error = desired - angle;
  float right_error = angle - desired;
  if (right_error < 0){
    right_error += 2 * PI;
  }
  if (left_error < 0){
    left_error += 2 * PI;
  }
  if (right_error < left_error){
    return right_error;
  }
  else {
    return -left_error;
  }
}

float standard_error(float angle, float desired){
  return desired - angle;
}

float PID_it(float current, float desired, float* I_prev, float (*error_func)(float,float) ){

  float umin = 0;
  float umax = 1;
  // TODO: fix the sample rate variable
  /*float Ts = 1.f / CONTROL_FREQ;*/
  float Ts = 1.f;
  float Ti = 0.02;
  float K = 0.00015;
  float error = error_func(current, desired);
  /*float error = desired - current;*/
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
  *I_prev = I;
  return u;
  // for some time
}
