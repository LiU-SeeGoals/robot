#include "motor.h"

void startMotor(MotorPWM *motor){
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

/*
    Resets the breaking pin
*/
void runMotor(MotorPWM *motor)
{
    HAL_GPIO_WritePin(motor->breakPinPort, motor->breakPin, GPIO_PIN_RESET);
}

/*
    Sets the breaking pin
*/
void breakMotor(MotorPWM *motor)
{
    HAL_GPIO_WritePin(motor->breakPinPort, motor->breakPin, GPIO_PIN_SET);
}

/*
    Reverses motor direction and makes sure that the motor is stopped before reversing
*/

void changeDirection(MotorPWM *motor, int percent)
{
    if ((motor->reversing && percent >= 0) || (!motor->reversing && percent <= 0))
    {
        while(readSpeed(motor) >= 0){
            breakMotor(motor);
        }
        if (percent < 0)
        {
            motor->reversing = 1;
        }
        else
        {
            motor->reversing = 0;
        }
        HAL_GPIO_WritePin(motor->breakPinPort, motor->reversePin, motor->reversing);
    }
}

/*
    Set speed of motor in percent 0 - 100
    Negative values are interpreted as reverse
*/
void setSpeed(MotorPWM *motor, float percent)
{

    changeDirection(motor, percent);
    runMotor(motor);

    // TODO: How to handle rounding errors, do they even matter?
    uint32_t pwm_speed = motor->htim->Init.Period * percent;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_speed);
}

int readSpeed(MotorPWM *motor)
{
    return HAL_GPIO_ReadPin(motor->readSpeedPinPort, motor->readSpeedPin);
}
