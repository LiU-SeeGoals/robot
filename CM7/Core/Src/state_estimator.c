#include "HandmadeMath.h"
#include "imu.h"
#include "state_estimator.h"
#include <stdio.h>
#include "log.h"


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

static robot_state robot;
const float imu_hz = 333.33;

static LOG_Module internal_log_mod;

void STATE_Init(){

  robot.acc_bias.x = 0;
  robot.acc_bias.y = 0;
  robot.acc_bias.z = 0;

  robot.gyr_bias.x = 0;
  robot.gyr_bias.y = 0;
  robot.gyr_bias.z = 0;


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
  robot.is_calibrated = -1;

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
  float T = 1.0 / imu_hz;
  Mat2 F = {1, T,
            0, 1};
  F = TransposeM2(F);

  Mat2 G = {T*T/2, 0,
            0,     T};

  G = TransposeM2(G);

  Mat2 q = {0.1, 0,
            0, 0.1};
  q = TransposeM2(q);

  Mat2 Q = MulM2(G, MulM2(q ,TransposeM2(G)));
  time_update_vec2(F, Q, P, x, B, u);
}

void STATE_calibrate_imu_gyr()
{
  const int calib_size = 1;

  float acc_bias_x = 0;
  float acc_bias_y = 0;
  float acc_bias_z = 0;

  float gyr_bias_x = 0;
  float gyr_bias_y = 0;
  float gyr_bias_z = 0;
  
  IMU_AccelVec3 acc;
  IMU_GyroVec3 gyr;
  for (int i = 0; i < calib_size; i++)
  {
    HAL_Delay(3);
    /*while(blocks_read == 0)*/
    /*{*/
    /*blocks_read = IMU_read_fifo_raw(imu_buf, buf_size);*/
    /*}*/

    gyr = IMU_read_gyro();
    acc = IMU_read_accel_mps2();

    // Read one block and extract gyro and accelerometer
    acc_bias_x += acc.x;
    acc_bias_y += acc.y;
    acc_bias_z += acc.z;

    gyr_bias_x += gyr.x;
    gyr_bias_y += gyr.y;
    gyr_bias_z += gyr.z;

    /*acc_bias_x += IMU_RAW_TO_MPS2(imu_buf[0].accel.x);*/
    /*acc_bias_y += IMU_RAW_TO_MPS2(imu_buf[0].accel.y);*/
    /*acc_bias_z += IMU_RAW_TO_MPS2(imu_buf[0].accel.z);*/
    /**/
    /*gyr_bias_x += IMU_RAW_TO_DPS(imu_buf[0].gyro.x);*/
    /*gyr_bias_y += IMU_RAW_TO_DPS(imu_buf[0].gyro.y);*/
    /*gyr_bias_z += IMU_RAW_TO_DPS(imu_buf[0].gyro.z);*/
  }

  robot.acc_bias.x = acc_bias_x / ((float) calib_size);
  robot.acc_bias.y = acc_bias_y / ((float) calib_size);
  robot.acc_bias.z = acc_bias_y / ((float) calib_size);
                                                       
  robot.gyr_bias.x = gyr_bias_x / ((float) calib_size);
  robot.gyr_bias.y = gyr_bias_y / ((float) calib_size);
  robot.gyr_bias.z = gyr_bias_y / ((float) calib_size);

  robot.is_calibrated = 1;
  LOG_INFO("Done calibrating\r\n");
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

void STATE_gyr_measure(IMU_GyroVec3 gyr)
{

}

void STATE_acc_measure()
{
    IMU_AccelVec3 acc = IMU_read_accel_mps2();

    float T = 1.0 / imu_hz;
    Mat2 B = {T*T/2, 0,
                T,   0};

    B = TransposeM2(B);

    Vec2 accmejure_x = {acc.x - robot.acc_bias.x, 0};
    Vec2 accmejure_y = {acc.y - robot.acc_bias.y, 0};

    /*printf("cv update no acceleration speed %f\n", robot.statex.Y);*/
    /*cv_update_vec2(&robot.Px, &robot.statex, Bnone, accmejure);*/
    cv_update_vec2(&robot.Px, &robot.statex, B, accmejure_x);
    cv_update_vec2(&robot.Py, &robot.statey, B, accmejure_y);

    /*LOG_INFO("acc x=%f y=%f\r\n", accmejure_x.X, accmejure_y.X);*/
    /*LOG_INFO("real acc x=%f y=%f\r\n", acc.x, acc.y);*/
    /*LOG_INFO("bias_acc x=%f y=%f\r\n", robot.acc_bias.x, robot.acc_bias.y);*/
    /*LOG_INFO("x=%f y=%f w=%f\r\n", robot.statex.X, robot.statey.X, robot.statew.X);*/
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

int STATE_is_calibrated(){
  return robot.is_calibrated;
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
