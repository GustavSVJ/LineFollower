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

extern char MoveTo(move_t *directions);
extern void InitializeMotors();


#endif /* MotorControl_H_ */
