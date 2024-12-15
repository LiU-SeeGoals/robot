#ifndef POSFOLLOW_H
#define POSFOLLOW_H

typedef struct{
  float umin;
  float umax;
  float Ts;
  float Ti;
  float K;

} control_params;

float PID_it(float current, float desired, float* I_prev, float (*error_func)(float,float), control_params param);

float angle_error(float angle, float desired);

float standard_error(float current, float desired);


void set_params();


void go_to_position(Vec2 desired_pos, float wantw);



float PID_it(float current, float desired, float* I_prev, float (*error_func)(float,float), control_params param);

#endif /* COM_H */
