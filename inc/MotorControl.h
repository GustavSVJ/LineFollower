#ifndef MotorControl_H_
#define MotorControl_H_

#include "stm32f30x_conf.h" // STM32 config

typedef struct {
    int angle;
    uint32_t distance;
    uint32_t speed;
} move_t;

extern void InitializeMotorTimer(int topValue, int prescaler);

extern void InitializeLeftMotor();
extern void SetDutycycleLeftMotor(int dutycycle);

extern void InitializeRightMotor();
extern void SetDutycycleRightMotor(int dutycycle);

extern void InitializeRightMotorEncoder();
extern void EnableRightMotorEncoder();
extern void DisableRightMotorEncoder();

extern void InitializeLeftMotorEncoder();
extern void EnableLeftMotorEncoder();
extern void DisableLeftMotorEncoder();


#endif /* MotorControl_H_ */
