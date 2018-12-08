#include "stm32f30x_conf.h" // STM32 config
#include "PID.h"
#include "Uart.h"
#include "Regulator.h"
#include "MotorControl.h"
#include "UcamFunction.h"


#include <stdio.h>

#define CM_PER_STEP 0.2425089067f
#define HEADING_FACTOR 0.7679448714f

void Timer15_Init(uint16_t top, uint16_t prescaler);
void Timer15_InterruptEnable();

void HeadingUpdate();
void RightSpeedUpdate(double heading);
void LeftSpeedUpdate(double heading);

volatile char timer15_PIDFlag = 0;
volatile uint16_t leftTimeCounter = 0;
volatile uint16_t rightTimeCounter = 0;

static double b0 = 10.85;
static double b1 = -10.67;
static double a1 = -1.0;

static double head_b0 = 0.02194;
static double head_b1 = 0.0217;
static double head_a1 = 0.9641;

volatile double rightSpeed = 0;
volatile double leftSpeed = 0;

volatile uint16_t leftMotorTotalPulses = 0;
volatile uint16_t rightMotorTotalPulses = 0;

volatile int16_t leftTurnGoal = 0;
volatile int16_t rightTurnGoal = 0;

uint16_t rightRef = 0;
uint16_t leftRef = 0;
int16_t headingRef = 0;


statetype rightMotorRegul;
statetype leftMotorRegul;
statetype headingRegul;

void RegulatorSetRefs(uint16_t rightReference, uint16_t leftReference, int16_t headingReference){
    rightRef = rightReference;
    leftRef = leftReference;
    headingRef = headingReference;
}

void DriveTo(uint16_t distance, int16_t angle, uint16_t speed){
    headingRef = angle;

    leftMotorTotalPulses = 0;
    rightMotorTotalPulses = 0;

    uint16_t running = 0;

    if (angle < 0){
        leftTurnGoal = angle / HEADING_FACTOR;
        RegulatorSetRefs(0,speed,0);
        running = 1;
    }
    else if(angle > 0){
        rightTurnGoal = angle / HEADING_FACTOR;
        RegulatorSetRefs(speed,0,0);
        running = 1;
    }

    while(running){

        if (timer15_PIDFlag){

            timer15_PIDFlag = 0;
            if (angle < 0){
                LeftSpeedUpdate(0);
            }
            else{
                RightSpeedUpdate(0);
            }
        }

        if (leftTurnGoal == 0 && rightTurnGoal == 0){
            SetDutycycleRightMotor(0);
            SetDutycycleLeftMotor(0);
            running = 0;
        }

    }

    delay_ms(300);

    uint32_t leftMotorGoal = 0;
    uint32_t rightMotorGoal = 0;

    if (distance > 0){
        running = 1;

        leftMotorGoal = leftMotorTotalPulses + (distance / CM_PER_STEP);
        rightMotorGoal = rightMotorTotalPulses + (distance / CM_PER_STEP);
        RegulatorSetRefs(speed,speed,headingRef);
    }

    while(running){

        if (timer15_PIDFlag){

            timer15_PIDFlag = 0;
            HeadingUpdate();
            RightSpeedUpdate(headingRegul.u);
            LeftSpeedUpdate(headingRegul.u);
        }

        if (leftMotorTotalPulses > leftMotorGoal || rightMotorTotalPulses > rightMotorGoal){
            running = 0;
            SetDutycycleRightMotor(0);
            SetDutycycleLeftMotor(0);
        }
    }
}

void HeadingUpdate(){

    double heading = ((double)rightMotorTotalPulses - (double)leftMotorTotalPulses) * HEADING_FACTOR;

    double e = headingRef - heading;
    regul_out(&headingRegul, e, head_b0);
    regul_update(&headingRegul, e, head_b1, head_a1);

    if (headingRegul.u > 1){
        headingRegul.u = 1;
    }
    else if (headingRegul.u < -1){
        headingRegul.u = -1;
    }

}


void RightSpeedUpdate(double heading){

    double e  = (rightRef * (heading + 1)) - rightSpeed;
    regul_out(&rightMotorRegul, e, b0);
    regul_update(&rightMotorRegul, e, b1, a1);

    if (rightMotorRegul.u > 600){
        SetDutycycleRightMotor(600);
    }
    else if(rightMotorRegul.u < 0){
        SetDutycycleRightMotor(0);
    }
    else{
        uint16_t output = (unsigned int)rightMotorRegul.u;
        SetDutycycleRightMotor(output);
    }

}

void LeftSpeedUpdate(double heading){
    double e = (leftRef * ((-1 * heading) + 1)) - leftSpeed;
    regul_out(&leftMotorRegul, e, b0);
    regul_update(&leftMotorRegul, e, b1, a1);

    if (leftMotorRegul.u > 600){
        SetDutycycleLeftMotor(600);
    }
    else if(leftMotorRegul.u < 0){
        SetDutycycleLeftMotor(0);
    }
    else{
        uint16_t output = (unsigned int)leftMotorRegul.u;
        SetDutycycleLeftMotor(output);
    }

}

void RegulatorInit(){

regul_init(&rightMotorRegul);
regul_init(&leftMotorRegul);

}

void RegulatorRun(){
    RegulatorInit();
    InitializeMotors(600, 1);
    Timer15_Init(15999,3);
    Timer15_InterruptEnable();

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_8;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioStructure);


}

void Timer15_Init(uint16_t top, uint16_t prescaler){

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    TIM_TimeBaseStructInit (&timerInitStructure);
    timerInitStructure.TIM_Prescaler = prescaler;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = top;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM15, &timerInitStructure);
    TIM_Cmd(TIM15, ENABLE);

    void regul_init(statetype *p);


}

void Timer15_InterruptEnable()
{
    TIM_ITConfig(TIM15,TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvicStructure;

    nvicStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    NVIC_SetPriority(TIM2_IRQn, 0);

    TIM_ITConfig(TIM15,TIM_IT_Update, ENABLE);
}

void Timer15_InterruptDisable()
{
    TIM_ITConfig(TIM15,TIM_IT_Update, DISABLE);
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
    static uint16_t i = 0;
    if (TIM_GetITStatus(TIM15, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM15, TIM_IT_Update);

        if (i > 9){
            timer15_PIDFlag = 1;
            i = 1;
        }
        else{
            i++;
        }

        if (leftTimeCounter > 100){
            leftSpeed = 0;
        }
        else{
            leftTimeCounter++;
        }

        if (rightTimeCounter > 100){
            rightSpeed = 0;
        }
        else{
            rightTimeCounter++;
        }


    }
}

void EXTI9_5_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) { //Check if interrupt flag is set
        EXTI_ClearITPendingBit(EXTI_Line5); //Clear interrupt flag

        leftMotorTotalPulses++;

        if (leftTimeCounter > 100){
            leftSpeed = 0;
            leftTimeCounter = 1;
        }
        else{
            leftSpeed = CM_PER_STEP / (leftTimeCounter / 1000.0);
            leftTimeCounter = 1;
        }

        if (leftTurnGoal < 0){
            leftTurnGoal++;
        }


	}
}

void EXTI4_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) { //Check if interrupt flag is set
        EXTI_ClearITPendingBit(EXTI_Line4); //Clear interrupt flag

        rightMotorTotalPulses++;

        if (rightTimeCounter > 100){
            rightSpeed = 0;
            rightTimeCounter = 1;
        }
        else{
            rightSpeed = CM_PER_STEP / (rightTimeCounter / 1000.0);
            rightTimeCounter = 1;
        }

        if (rightTurnGoal > 0){
            rightTurnGoal--;
        }
	}


}


