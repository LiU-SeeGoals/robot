#include "HandmadeMath.h"
#include "state_estimator.h"
#include <stdio.h>
#include "log.h"

robot_state robot;

// state vector 
// x = [px, py, vx, vy, angle, omega]
// Dont do any constant velocity updates for the angle state, as it does not make sense
// Some kind of angle model if needed if you want motion model for the angle aswell

// State-transition (constant veloctiy model with angle) 
// F = [1 0 T 0;
//      0 1 0 T
//      0 0 1 T]
//      0 0 0 1]
// Measurement matrix
// H = [1; 1; 0; 0; 1; 0;]
// For sensor model y = Hx

// Time update with accelerometer
// B = [T^2/2;
//      T^2/2;
//      T;
//      T;
//      0;
//      0]
// U = [accx, accy, accx, accy, 0, 0]

// Time update with gyro
// B = [0;
//      0;
//      0;
//      0;
//      T^2/2;
//      T]
// U = [0, 0, 0, 0, gyrx, gyry]

//     T^2/2 0
// G = 0     T^2/2
//     T     0
//     0     T
//     0     0

// q = [1 0 0 0;
//      0 1 0 0
//      0 0 1 0
//      0 0 0 1]
// Q = GqG^T
// q is just to make tuning easier

// This gives motion model x = Fx + Gu + w
// where w is process noise


// The code uses the notation of the course sensor fusion
// It is almost the same for every instance of kalman filter
// go to wikipedia and you will find the same notation. in wikipedia z is mejurement

// Matrices are coloum major instead of row major
// This means that matrices act as if they were transposed when multiplying
// which is why you will see every matrix be transposed before intialization

static LOG_Module internal_log_mod;

void STATE_Init(){

  robot.statex.X = 0;
  robot.statex.Y = 0;

  robot.statey.X = 0;
  robot.statey.Y = 0;

  robot.statew.X = 0;
  robot.statew.Y = 0;

  robot.Px.Elements[0][0] = 1;
  robot.Px.Elements[1][0] = 0;
  robot.Px.Elements[0][1] = 0;
  robot.Px.Elements[1][1] = 1;

  robot.Py.Elements[0][0] = 1;
  robot.Py.Elements[1][0] = 0;
  robot.Py.Elements[0][1] = 0;
  robot.Py.Elements[1][1] = 1;

  robot.Pw.Elements[0][0] = 1;
  robot.Pw.Elements[1][0] = 0;
  robot.Pw.Elements[0][1] = 0;
  robot.Pw.Elements[1][1] = 1;

  robot.is_initiated = -1;

  LOG_InitModule(&internal_log_mod, "STATE", LOG_LEVEL_TRACE);
}

void measurement_update_vec2_1d(Mat2* P, float R, Vec2 H, float mejure, Vec2* x)
{
  float y = mejure - DotV2(H,*x);

  float S = P->Columns[0].X + R;
  Vec2 K = MulV2F(P->Columns[0], 1/S);
  Mat2 KH = {K.X, 0, 
             K.Y, 0};

  KH = TransposeM2(KH);

  *x = AddV2(*x, MulV2F(K, y));
  *P = SubM2(*P, MulM2(KH, *P));
}

void time_update_vec2(Mat2 F,  Mat2 Q, Mat2* P, Vec2* x, Mat2 B, Vec2 u)
{
  *x = AddV2(MulM2V2(F, *x), MulM2V2(B, u));
  *P = AddM2(MulM2(F, MulM2(*P, TransposeM2(F))), Q);
}

void cv_update_vec2(Mat2* P, Vec2* x, Mat2 B, Vec2 u) {
  float T = 1;
  Mat2 F = {1, T,
            0, 1};
  F = TransposeM2(F);

  Mat2 G = {T*T/2, 0,
            0,     T};

  G = TransposeM2(G);

  Mat2 q = {1, 0,
            0, 1};
  q = TransposeM2(q);

  Mat2 Q = MulM2(G, MulM2(q ,TransposeM2(G)));
  time_update_vec2(F, Q, P, x, B, u);
}

void camera_meas(float posx, float posy, float angle){
    float R = 1;
    Vec2 H = {1, 0};
    measurement_update_vec2_1d(&robot.Px, R, H, posx, &robot.statex);
    measurement_update_vec2_1d(&robot.Py, R, H, posy, &robot.statey);
    measurement_update_vec2_1d(&robot.Pw, R, H, angle, &robot.statew);

    robot.statex.X = posx;
    robot.statey.X = posy;
    robot.statew.X = angle;
}

float get_robot_angle() {

  if (robot.statew.X < 0){
    robot.statew.X += 2 * PI;
  }
  if (robot.statew.X > 2 * PI){
    robot.statew.X -= 2 * PI;
  }
  return robot.statew.X;
}

void initialize_kalman(float x, float y, float w){
  robot.statex.X = x;
  robot.statey.X = y;
  robot.statew.X = w;

  robot.statex.Y = 0;
  robot.statey.Y = 0;
  robot.statew.Y = 0;

  robot.is_initiated = 1;
}

/*
  @Return -1 if not inited else 1 
*/
int kalman_is_initiated(){
  return robot.is_initiated;
}

float get_angle_vel(){
  return robot.statew.Y;
}

float get_posx(){
  return robot.statex.X;
}

float get_posy(){
  return robot.statey.X;
}

float get_vx(){
  return robot.statex.Y;
}

float get_vy(){
  return robot.statey.Y;
}
