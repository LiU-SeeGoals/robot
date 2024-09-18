#include "../Inc/HandmadeMath.h"
#include <stdio.h>

// Each state is for one dimeion
// state x is x position and velocity
// state y is y position and velocity
// state w is angle and angle-velocity

Vec2 statex = {2,1};
Vec2 statey = {2,1};
Vec2 statew = {2,1};

Mat2 Px = {1,0,
           0,1};
Mat2 Py = {1,0,
           0,1};
Mat2 Pw = {1,0,
           0,1};

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

void measurement_update_vec2(Mat2 H, Mat2* P, Mat2 R, Vec2 mejurement, Vec2* x)
{
  Vec2 y = SubV2(mejurement, MulM2V2(H, *x));

  Mat2 S = AddM2(MulM2(H, MulM2(*P, TransposeM2(H))), R);
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

// Do time update for pos, vel, angle 
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

void do_some_kalman() {
    float T = 1;

    Mat2 Bnone = {0,0,0,0};
    Mat2 B = {T*T/2, 0,
                0,   T};
    B = TransposeM2(B);

    Vec2 accmejure = {1, 0};

    printf("cv update no acceleration\n");
    cv_update_vec2(&Px, &statex, Bnone, accmejure);
    printv2(&statex);
    printf("cv update with acceleration 1\n");
    cv_update_vec2(&Px, &statex, B, accmejure);
    printv2(&statex);

    Mat2 H = {1, 0, 
              0, 1};
    H = TransposeM2(H);

    Mat2 R = {1, 0,
              0, 1};
    H = TransposeM2(R);
    Vec2 posmejure = {1, 0};
    measurement_update_vec2(H, &Px, R, posmejure, &statex);
    printf("measurement update\n");
    printf("states\n");
    printv2(&statex);
    printf("hello world\n");
}
