#include "HandmadeMath.h"
#include "state_estimator.h"
#include <stdio.h>


robot_state robot;

// Constant velocity model
// state space model
// x = [px py w vx vy dw]
//
//     1 0 T 0 0
// F = 0 1 0 T 0
//     0 0 1 0 0
//     0 0 0 1 0
//     0 0 0 0 1

//     T^2/2 0
// G = 0     T^2/2
//     T     0
//     0     T
//     0     0
// Q = GqG^T
// q tuning param

// acc mesasurement
// y = H x ( + e )
// y: ax, ay
//
//     T^2/2 0     0 0 0
// H = 0     T^2/2 0 0 0
//     0     0     T 0 0
//     0     0     0 T 0
//     0     0     0 0 0

// gyr mesasurement
// y = H x ( + e )
// y: ax, ay
//
//     0 0 0 0 0
// H = 0 0 0 0 0
//     0 0 0 0 0
//     0 0 0 0 0
//     0 0 0 0 T

// cam mesasurement
// y = H x ( + e )
// y: ax, ay
//
//     1 0 0 0 0
// H = 0 1 0 0 0
//     0 0 0 0 0
//     0 0 0 0 0
//     0 0 0 0 0


// The code uses the notation of the course sensor fusion
// It is almost the same for every instance of kalman filter
// go to wikipedia and you will find the same notation. in wikipedia z is mejurement

// Matrices are coloum major instead of row major
// This means that matrices act as if they were transposed when multiplying
// which is why you will see every matrix be transposed before intialization
// This also means if you wanna visualize a matrix you should transpose it before printing

void init(Mat2* P, float R, float mejurement, Vec2* x){
  Vec2 statex = {2,1};
  Vec2 statey = {2,1};
  Vec2 statew = {2,1};

  Mat2 Px = {1,0,
             0,1};
  Mat2 Py = {1,0,
             0,1};
  Mat2 Pw = {1,0,
             0,1};

  robot.statex = statex;
  robot.statey = statex;
  robot.statew = statex;

  robot.Px = Px;
  robot.Py = Py;
  robot.Pw = Pw;
}


void measurement_update_vec2_1d(Mat2* P, float R, float mejurement, Vec2* x)
{
  /*Vec2 y = SubV2(mejurement, MulV2(H, *x));*/
  float y = mejurement - x->X;

  /*Vec2 Sxy = Add(MulV2(H, MulM2V2(*P, H)), R);*/

  float S = P->Columns[0].X + R;
  /*printf("determinant %f\n", DeterminantM2(S));*/
  /*Mat2 K = MulM2(*P, MulM2(TransposeM2(H), InvGeneralM2(S)));*/
  Vec2 K = MulV2F(P->Columns[0], 1/S);
  /**x = AddV2(*x, MulM2V2(K, y));*/
  Mat2 KH = {K.X, 0, 
             K.Y, 0};
  KH = TransposeM2(KH);

  *x = AddV2(*x, MulV2F(K, y));
  *P = SubM2(*P, MulM2(KH, *P));
}

void measurement_update_vec2(Mat2 H, Mat2* P, Mat2 R, Vec2 mejurement, Vec2* x)
{
  Vec2 y = SubV2(mejurement, MulM2V2(H, *x));

  Mat2 S = AddM2(MulM2(H, MulM2(*P, TransposeM2(H))), R);
  printf("determinant %f\n", DeterminantM2(S));
  Mat2 K = MulM2(*P, MulM2(TransposeM2(H), InvGeneralM2(S)));
  *x = AddV2(*x, MulM2V2(K, y));
  *P = SubM2(*P, MulM2(K,MulM2(H, *P)));
}

void measurement_update_vec3(Mat3 H, Mat3 P, Mat3 R, Vec3 mejurement, Vec3 x)
{
  Mat3 S = AddM3(MulM3(H, MulM3(P, TransposeM3(H))), R);
  Mat3 K = MulM3(P, MulM3(TransposeM3(H), S));
  Vec3 y = SubV3(mejurement, MulM3V3(H, x));
  Vec3 x_1 = AddV3(x, MulM3V3(K, y));
}

void time_update_vec2(Mat2 F,  Mat2 Q, Mat2* P, Vec2* x, Mat2 B, Vec2 u)
{
  *x = AddV2(MulM2V2(F, *x), MulM2V2(B, u));
  *P = AddM2(MulM2(F, MulM2(*P, TransposeM2(F))), Q);
}

Vec3 time_update_vec3(Mat3 F, Mat3 Q, Vec3* x)
{
  Vec3 x_hat = MulM3V3(F, *x);
  Mat3 P = AddM3(MulM3(F, MulM3(P, TransposeM3(F))), Q);
  return x_hat;
}

void printm2(Mat2 a){

  printf("----------------\n");
  printf("%f %f\n", a.Columns[0].X, a.Columns[0].Y);
  printf("%f %f\n", a.Columns[1].X, a.Columns[1].Y);
  printf("----------------\n");

}
void printm3(Mat3 a){

  printf("----------------\n");
  printf("%f %f %f\n", a.Columns[0].X, a.Columns[0].Y, a.Columns[0].Z);
  printf("%f %f %f\n", a.Columns[1].X, a.Columns[1].Y, a.Columns[1].Z);
  printf("%f %f %f\n", a.Columns[2].X, a.Columns[2].Y, a.Columns[2].Z);
  printf("----------------\n");

}

void printv2(Vec2* a){
    printf("%f %f\n" , a->X, a->Y);
}

void printv3(Vec3* a){
    printf("%f %f %f\n" , a->X, a->Y, a->Z);
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

void cv_update_vec3(Vec3* x) {
  float T = 1;
  Mat3 F = {1, T, 0, 
            0, 1, 0, 
            0, 0, 1};
  F = TransposeM3(F);
  printm3(F);

  Mat3 G = {T*T/2, 0, 0, 
            0,     T, 0, 
            0,     0, 1};
  G = TransposeM3(G);
  Mat3 q = {1, 0, 0, 
            0, 1, 0, 
            0, 0, 1};
  q = TransposeM3(q);
  Mat3 Q = MulM3(G, MulM3(q ,TransposeM3(G)));
  time_update_vec3(F, Q, x);
}


void camera_meas(float posx, float posy){
    float R = 1;
    measurement_update_vec2_1d(&robot.Px, R, posx, &robot.statex);
    measurement_update_vec2_1d(&robot.Py, R, posy, &robot.statey);
}

void do_some_kalman() {
    float T = 1;

    Mat2 Bnone = {0,0,0,0};
    Mat2 B = {T*T/2, 0,
                T,   0};
    B = TransposeM2(B);

    Vec2 accmejure = {1, 0};

    Mat2 H = {T*T/2, T, 0, T};
    H = TransposeM2(H);

    printf("cv update no acceleration\n");
    cv_update_vec2(&robot.Px, &robot.statex, Bnone, accmejure);
    printv2(&robot.statex);
    printf("cv update with acceleration 1\n");
    cv_update_vec2(&robot.Px, &robot.statex, B, accmejure);
    printv2(&robot.statex);
    float posmejure = 8;
    printf("covariance\n");
    printm2(robot.Px);
    float R = 1;
    /*Mat2 H = {2/(T * T)}*/
    /*measurement_update_vec2(H, &Px, R, posmejure, &statex);*/
    measurement_update_vec2_1d(&robot.Px, R, posmejure, &robot.statex);
    cv_update_vec2(&robot.Px, &robot.statex, Bnone, accmejure);
    /*measurement_update_vec2_1d(&Px, R, posmejure, &statex);*/
    /*cv_update_vec2(&Px, &statex, Bnone, accmejure);*/
    /*measurement_update_vec2_1d(&Px, R, posmejure, &statex);*/
    /*cv_update_vec2(&Px, &statex, Bnone, accmejure);*/
    /*measurement_update_vec2_1d(&Px, R, posmejure, &statex);*/
    /*cv_update_vec2(&Px, &statex, Bnone, accmejure);*/
    /*measurement_update_vec2_1d(&Px, R, posmejure, &statex);*/
    /*cv_update_vec2(&Px, &statex, Bnone, accmejure);*/
    printf("measurement update\n");
    printm2(robot.Px);
    printv2(&robot.statex);
    printf("cov\n");
    printm2(robot.Px);
    printf("hello world\n");
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
