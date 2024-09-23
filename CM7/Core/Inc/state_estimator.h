#include "../Inc/HandmadeMath.h"
#include <stdio.h>

/*// State is x, vx*/
/*Vec2 statex = {2,1};*/
/*Vec2 statey = {2,1};*/
/*Mat2 Px = {0,0,*/
/*           0,0};*/
/*Mat2 Py = {0,0,*/
/*           0,0};*/

typedef struct {

  Vec2 statex;
  Vec2 statey;
  Vec2 statez;

  Mat2 Px;
  Mat2 Py;
  Mat2 Pz;

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
// Only the state estimator should write to
// the state variables
float get_robot_angle();
float get_angle_vel();
float get_posx();
float get_posy();
float get_vx();
float get_vy();

