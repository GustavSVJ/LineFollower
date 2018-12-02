#ifndef PID_H_
#define PID_H_

extern volatile char timer15_PIDFlag;
extern volatile uint32_t leftMotorTotalPulses;
extern volatile uint32_t rightMotorTotalPulses;

void RegulatorRun();
void Timer15_InterruptEnable();
void Timer15_InterruptDisable();
void RegulatorSetRefs(uint16_t rightReference, uint16_t leftReference, int16_t headingReference);
void RegulatorUpdate();
void DriveTo(uint16_t distance, int16_t angle, uint16_t speed);

#endif /* PID_H_ */
