#ifndef MotorControl_H_
#define MotorControl_H_

#include "stm32f30x_conf.h" // STM32 config

void InitializeMotors(uint16_t top, uint16_t prescaler);

void SetDutycycleLeftMotor(int dutycycle);
void SetDutycycleRightMotor(int dutycycle);






#endif /* MotorControl_H_ */
