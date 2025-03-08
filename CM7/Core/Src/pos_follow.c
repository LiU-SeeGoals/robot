#include "HandmadeMath.h"
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
  LOG_InitModule(&internal_log_mod, "POS", LOG_LEVEL_TRACE, 0);
}

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

float standard_error(float current, float desired) {
  return desired - current;
}

void set_params() {

  params_angle.umin = -1000.0;
  params_angle.umax = 1000.0;
  params_angle.Ts = DELTA_T;
  params_angle.Ti = 10000000;
  params_angle.Td = 0;
  params_angle.K = 8;

  params_dist.umin = -1000.0;
  params_dist.umax = 1000.0;
  params_dist.Ts = DELTA_T;
  params_dist.Ti = 0.0015;
  params_dist.K = 0.015 * 2;
  params_dist.Td = 0.1;
}


void TEST_vy(float ref_angle, float speed) 
{
  float control_w = PID_it(STATE_get_robot_angle(), ref_angle, &angle_I, angle_error, &params_angle);
  steer(0, speed, -control_w);
}

void TEST_vx(float ref_angle, float speed) 
{
  float control_w = PID_it(STATE_get_robot_angle(), ref_angle, &angle_I, angle_error, &params_angle);
  steer(speed, 0.0f, -control_w);
}

void TEST_angle_control(float ref_angle)
{
  float control_w = PID_it(STATE_get_robot_angle(), ref_angle, &angle_I, angle_error, &params_angle);
  steer(0, 0, -control_w);
}

void POS_go_to_position(float dest_x, float dest_y, float wantw) {

  Vec2 current_pos = {STATE_get_posx(), STATE_get_posy()};
  Vec2 desired_pos = {dest_x, dest_y};

  float angle = STATE_get_robot_angle();
  Vec2 relative_pos = SubV2(current_pos, desired_pos); 
  float euclidian_distance = relative_pos.X * relative_pos.X + relative_pos.Y * relative_pos.Y;

  float distance_control_signal = PID_it(euclidian_distance, 0.0, &dist_I, standard_error, &params_dist);

  float control_w = PID_it(STATE_get_robot_angle(), wantw, &angle_I, angle_error, &params_angle);

  // The steering signal is a velocity, so calculate how much of each component we need

  // Rotate from football field to robot coordinates
  /*[cos(theta), -sin(theta)]*/
  /*[sin(theta), cos(theta)]*/
  float x = (relative_pos.X * cos(angle)) - (relative_pos.Y * sin(angle));
  float y = (relative_pos.X * sin(angle)) + (relative_pos.Y * cos(angle));

  steer(100.f * x, 100.f * y, -control_w);
}

float prev_error = 0;

float PID_it(float current, float desired, float* I_prev, float (*error_func)(float,float), control_params *param){

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
