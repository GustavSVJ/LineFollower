#ifndef PID_H_
#define PID_H_

extern volatile char timer15_PIDFlag;

void RegulatorRun();
void Timer15_InterruptEnable();
void Timer15_InterruptDisable();
void RegulatorUpdate(uint16_t rightRef, uint16_t leftRef);

#endif /* PID_H_ */
