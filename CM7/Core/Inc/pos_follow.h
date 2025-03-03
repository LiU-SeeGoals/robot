#ifndef POSFOLLOW_H
#define POSFOLLOW_H

typedef struct{
  float umin;
  float umax;
  float Ts;
  float Ti;
  float Td;
  float K;

} control_params;

float angle_error(float angle, float desired);

float standard_error(float current, float desired);

void POS_set_params();

void POS_go_to_position(float dest_x, float dest_y, float wantw);

void TEST_angle_control(float ref_angle);

void POS_Init();

void TEST_vy(float ref_angle, float speed);
void TEST_vx(float ref_angle, float speed);
void TEST_angle_control(float ref_angle);

float PID_pi(float current, float desired, float* I_prev, float (*error_func)(float,float), control_params *param);
float PID_p(float current, float desired, float (*error_func)(float,float), control_params *param);
#endif /* COM_H */
