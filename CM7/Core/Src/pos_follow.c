#include "pos_follow.h"
#include "log.h"
#include "state_estimator.h"
#include "nav.h"

// Each state has an integration part in the pid controller
float dist_I = 0.01;
float angle_I = 0.01;

control_params params_dist;
control_params params_angle;

static robot_nav_command robot_nav;

const float DELTA_T = 0.001;

static LOG_Module internal_log_mod;

void POS_Init(){
  LOG_InitModule(&internal_log_mod, "POS", LOG_LEVEL_TRACE);
}

float angle_error(float angle, float desired){

  // TODO make sure returned sign is correct for the desired direction
  /*float left_error = desired - angle;*/
  /*float right_error = angle - desired;*/
  /*if (right_error < 0){*/
  /*  right_error += 2 * PI;*/
  /*}*/
  /*if (left_error < 0){*/
  /*  left_error += 2 * PI;*/
  /*}*/
  /*if (right_error < left_error){*/
  /*  return right_error;*/
  /*}*/
  /*else {*/
  /*  return -left_error;*/
  /*}*/
  return desired - angle;
}

float standard_error(float current, float desired) {
  return desired - current;
}

void set_params() {

  params_angle.umin = -100.0;
  params_angle.umax = 100.0;
  params_angle.Ts = DELTA_T;
  params_angle.Ti = 1000000000000;
  params_angle.Td = 0;
  params_angle.K = 8 * 180 / PI;

  params_dist.umin = -100.0;
  params_dist.umax = 100.0;
  params_dist.Ts = DELTA_T;
  params_dist.Ti = 0.0015;
  params_dist.K = 0.015 * 2;
  params_dist.Td = 0.1;
}


void TEST_vy(float ref_angle, float speed) 
{
  float control_w = PID_p(STATE_get_robot_angle(), ref_angle, angle_error, &params_angle);
  steer(0, speed, -control_w);
}

void TEST_vx(float ref_angle, float speed) 
{
  float control_w = PID_p(STATE_get_robot_angle(), ref_angle, angle_error, &params_angle);
  steer(speed, 0.0f, -control_w);
}

void TEST_angle_control(float ref_angle)
{
  float control_w = PID_p(STATE_get_robot_angle(), ref_angle, angle_error, &params_angle);
  steer(0, 0, -control_w);
}

int debug_print = 0;
void POS_go_to_position(float dest_x, float dest_y, float wantw) {

  float cur_x = STATE_get_posx();
  float cur_y = STATE_get_posy();
  float angle = STATE_get_robot_angle();

  float euclidian_distance = (dest_x - cur_x) * (dest_x - cur_x) + (dest_y - cur_y) * (dest_y - cur_y);

  float distance_control_signal = PID_pi(euclidian_distance, 0.0, &dist_I, standard_error, &params_dist);
  float control_w = PID_p(STATE_get_robot_angle(), wantw, angle_error, &params_angle);

  // Rotate from football field to robot coordinates
  /*[cos(theta), -sin(theta)]*/
  /*[sin(theta), cos(theta)]*/
  float x = (relative_pos.X * cos(angle)) - (relative_pos.Y * sin(angle));
  float y = (relative_pos.X * sin(angle)) + (relative_pos.Y * cos(angle));

  // Threshhold to make it less "shaky"
  if (fabs(control_w) < 0.5f)
  {
    /*steer(100.f * x, 100.f * y, 0.0f);*/
    steer(0.0f, 0.0f, -control_w);
  }
  else 
  {
    /*steer(100.f * x, 100.f * y, -control_w);*/
    steer(0.0f, 0.0f, 0.0f);
  }
  // Robot coordinates to world coordinates are reverse
  // so minus on the control signal
  debug_print += (1 % 100);
  if (debug_print == 0)
  {
    /*LOG_DEBUG("w %f\r\n", -control_w);*/
  }
  /*steer(100.f * x, 100.f * y, -control_w);*/

}

float prev_error = 0;

float PID_p(float current, float desired, float (*error_func)(float,float), control_params *param){

  set_params();

  float error = error_func(current, desired);
  // d not used
  /*float d = param->Td*(error - prev_error)/DELTA_T;*/

  float v = param->K * (error);
  float u = 0;

  if (v > param->umax) {
    u = param->umax;
  }

  else if (v < param->umin) {
    u = param->umin;
  }

  else {
    u = v;
  }

  prev_error = error;

  return u;
  // for some time
}

float PID_pi(float current, float desired, float* I_prev, float (*error_func)(float,float), control_params *param){

  set_params();

  float error = error_func(current, desired);
  float I = *I_prev + (param->Ts / param->Ti) * error;
  // d not used
  /*float d = param->Td*(error - prev_error)/DELTA_T;*/

  float feed_forward = 0.0;
  float v = param->K * (error + I);
  float u = 0;
  // integrator windup fix
  if (v < param->umin || v > param->umax){
    I = *I_prev;
  }

  if (v > param->umax) {
    u = param->umax;
  }

  else if (v < param->umin) {
    u = param->umin;
  }

  else {
    u = v;
  }

  prev_error = error;
  *I_prev = I;

  return u;
  // for some time
}
