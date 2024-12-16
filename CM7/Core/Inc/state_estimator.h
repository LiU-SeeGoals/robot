#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H
#include "HandmadeMath.h"
#include <stdio.h>

// Each state is for one dimension
// statex is vx position and velocity
// statey is vy position and velocity
// statew is angle and angle-velocity
// This might seem counter intuivative but
// The reason is that x is dependant on vx 
// and each dimension is independant of each other
// So ordering it this way makes it easier to handle
// Each matrix

// NOTE: maybe replace matrices with simple structs?
// will be annoying having to handroll all matrix mults
// but will be more intiuative and more effiecient

typedef struct {

  int is_initiated;
  Vec2 statex;
  Vec2 statey;
  Vec2 statew;

  Mat2 Px;
  Mat2 Py;
  Mat2 Pw;

} robot_state; 

void measurement_update_vec2(Mat2 H, Mat2* P, Mat2 R, Vec2 mejurement, Vec2* x);
void measurement_update_vec3(Mat3 H, Mat3 P, Mat3 R, Vec3 mejurement, Vec3 x);
void time_update_vec2(Mat2 F,  Mat2 Q, Mat2* P, Vec2* x, Mat2 B, Vec2 u);
Vec3 time_update_vec3(Mat3 F, Mat3 Q, Vec3* x);
void cv_update_vec2(Mat2* P, Vec2* x, Mat2 B, Vec2 u);
void cv_update_vec3(Vec3* x);

void do_some_kalman();

void printm2(Mat2 a);
void printm3(Mat3 a);
void printv2(Vec2* a);
void printv3(Vec3* a);

// Return different states,
// Consider each state variable private to the state_estimator
float get_robot_angle();
float get_angle_vel();
float get_posx();
float get_posy();
float get_vx();
float get_vy();
int kalman_is_initiated();

void camera_meas(float posx, float posy, float angle);
void initialize_kalman(float x, float y, float w);
#endif /* STATE_ESTIMATOR_H */
