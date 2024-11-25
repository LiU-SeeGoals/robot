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

#endif /* COM_H */
