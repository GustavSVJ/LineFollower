#ifndef MotorControl_H_
#define MotorControl_H_

#include "stm32f30x_conf.h" // STM32 config

typedef struct {
    int angle;
    uint32_t distance;
    uint32_t speed;
} move_t;

typedef struct {
    int leftSteps;
    int rightSteps;
    int speed;
} MoveSteps;

extern volatile int leftMotorPulseCounter;
extern volatile int rightMotorPulseCounter;

void SetDutycycleLeftMotor(int dutycycle);

void SetDutycycleRightMotor(int dutycycle);

extern char MoveTo(move_t *directions);
extern void InitializeMotors(uint16_t top, uint16_t prescaler);




#endif /* MotorControl_H_ */
