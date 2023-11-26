#ifndef __MOTOR_H
#define __MOTOR_H

typedef struct 
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t reversePin;
    GPIO_TypeDef reversePinPort;
    uint16_t breakPin;
    GPIO_TypeDef breakPinPort;
    uint16_t readSpeedPin;
    GPIO_TypeDef readSpeedPinPort;
    uint16_t reversing;
} MotorPWM;

void startMotor(MotorPWM *motor);
void setSpeed(MotorPWM *motor, float percent);
void breakMotor(MotorPWM *motor);
int readSpeed(MotorPWM *motor);

#endif
