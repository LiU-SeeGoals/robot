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

const float DELTA_T = 0.001;

static LOG_Module internal_log_mod;

void POS_Init(){
  LOG_InitModule(&internal_log_mod, "POS", LOG_LEVEL_TRACE);
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

  params_angle.umin = -100.0;
  params_angle.umax = 100.0;
  params_angle.Ts = DELTA_T;
  params_angle.Ti = 0.1;
  params_angle.K = 2;

  params_dist.umin = -1000.0;
  params_dist.umax = 1000.0;
  params_dist.Ts = DELTA_T;
  params_dist.Ti = 0.0015;
  params_dist.K = 0.015 * 2;
}


/*void TEST_vy(Vec2 desired_pos, Vec2 at_position, float wantw) {*/
/**/
/*  float control_w = PID_it(STATE_get_robot_angle(), ref_angle, &angle_I, angle_error, &params_angle);*/
/*  steer(0, 100.f, -control_w);*/
/*}*/
/**/
/*// Test if robot can go straight*/
/*void TEST_vx(Vec2 desired_pos, Vec2 at_position, float wantw) {*/
/**/
/*  float control_w = PID_it(STATE_get_robot_angle(), ref_angle, &angle_I, angle_error, &params_angle);*/
/*  steer(100.f, 0, -control_w);*/
/*}*/

void TEST_angle_control(float ref_angle)
{
  float control_w = PID_it(STATE_get_robot_angle(), ref_angle, &angle_I, angle_error, &params_angle);
  /*LOG_DEBUG("cw %f pw %f\r\n", control_w, STATE_get_robot_angle());*/
  steer(0, 0, control_w);
}

void go_to_position(Vec2 desired_pos, float wantw) {

  Vec2 current_pos = {STATE_get_posx(), STATE_get_posy()};

  float angle = STATE_get_robot_angle();
  Vec2 relative_pos = SubV2(current_pos, desired_pos); 
  float euclidian_distance = relative_pos.X * relative_pos.X + relative_pos.Y * relative_pos.Y;
  // We want the distance to be zero.
  LOG_DEBUG("===============distance================");
  float distance_control_signal = PID_it(euclidian_distance, 0.0, &dist_I, standard_error, &params_dist);
  // We want the angle to be the desired

  LOG_DEBUG("===============angle================");
  float control_w = PID_it(STATE_get_robot_angle(), wantw, &angle_I, angle_error, &params_angle);

  // The steering signal is a velocity, so calculate how much of each component we need

  // Rotate from football field to robot coordinates
  /*[cos(theta), -sin(theta)]*/
  /*[sin(theta), cos(theta)]*/
  float x = (relative_pos.X * cos(angle)) - (relative_pos.Y * sin(angle));
  float y = (relative_pos.X * sin(angle)) + (relative_pos.Y * cos(angle));


  /*Vec2 r = {cos(angle), sin(angle)};*/
  // Project the local coordinate vector unto the relative vector to get the desiered scaled contribute of each x and y axis
  /*LOG_DEBUG("dist : (%f)\r\n", euclidian_distance);*/
  /*LOG_DEBUG("rel pos : (%f)\r\n", relative_pos);*/
  /*LOG_DEBUG("distcondtorl : (%f)\r\n", distance_control_signal);*/
  /*LOG_DEBUG("angles : (%f,%f)\r\n", cos(angle),sin(angle));*/
  /*Vec2 projected =  MulV2F(relative_pos, distance_control_signal * DotV2(r, relative_pos) / euclidian_distance);*/

  LOG_DEBUG("angle (w): (%f)\r\n", angle);
  LOG_DEBUG("relative (x,y): (%f,%f)\r\n", relative_pos.X, relative_pos.Y);
  LOG_DEBUG("steering with (x,y,z): (%f,%f,%f)\r\n", 100.f * x, 100.f * y, -control_w);
  // Dont know why minus lol
  steer(100.f * x, 100.f * y, -control_w);
  /*steer(100.f * x, 0, control_w);*/
  /*steer(0, 0, -control_w);*/
}


float PID_it(float current, float desired, float* I_prev, float (*error_func)(float,float), control_params *param){

  set_params();

  float error = error_func(current, desired);
  /*LOG_DEBUG("error: (%f)\r\n", error);*/
  float I = *I_prev + (param->Ts / param->Ti) * error;

  /*LOG_DEBUG("I.prev: (%f)\r\n", *I_prev);*/
  /*LOG_DEBUG("I: (%f)\r\n", I);*/
  /*LOG_DEBUG("I adding: (%f)\r\n", (param->Ts / param->Ti)* error);*/

  /*LOG_DEBUG("I: (%f)\r\n", I);*/
  /*LOG_DEBUG("I prev: (%f)\r\n", *I_prev);*/
  float v = param->K * (error + I);
  /*LOG_DEBUG("v (v): (%f)\r\n", v);*/
  float u = 0;
  // integrator windup fix
  if (v < param->umin || v > param->umax){
    I = *I_prev;
    /*LOG_DEBUG("fixing IIIIIIii");*/
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

  /*LOG_INFO("DATAu:%f;\r\n", u);*/
  // HAL_Delay(1);
  *I_prev = I;

  /*LOG_DEBUG("v: (%f)\r\n", v);*/
  /*LOG_DEBUG("u: (%f)\r\n", u);*/
  return u;
  // for some time
}
