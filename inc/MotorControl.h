#ifndef MotorControl_H_
#define MotorControl_H_

extern void InitializeMotorTimer(int topValue, int prescaler);
extern void InitializeLeftMotor();
extern void InitializeRightMotor();
extern void SetDutycycleLeftMotor(int dutycycle);
extern void SetDutycycleRightMotor(int dutycycle);

#endif /* MotorControl_H_ */
