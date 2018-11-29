#include "MotorControl.h"
#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "FIFO.h"
#include <math.h>


#define PWM_Speed 600
#define MOVEMENT_QUEUE_LENGTH 100

#define CM_PER_STEP 0.4684831150
#define DEGRESS_PER_STEP 3.35526315812505

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))


MoveSteps movementData[MOVEMENT_QUEUE_LENGTH];
fifo_t movements;

volatile int leftMotorPulseCounter = 0;
volatile int leftMotorGoal = 57;
volatile int leftRunning = 0;
volatile int leftSpeed = PWM_Speed;

volatile int rightMotorPulseCounter = 0;
volatile int rightMotorGoal = 57;
volatile int rightRunning = 0;
volatile int rightSpeed = PWM_Speed;

int ConvertToSteps(move_t *directions);
void InitializeMotorTimer(int topValue, int prescaler);

void InitializeLeftMotor();
void SetDutycycleLeftMotor(int dutycycle);

void InitializeRightMotor();
void SetDutycycleRightMotor(int dutycycle);

void InitializeRightMotorEncoder();
void EnableRightMotorEncoder();
void DisableRightMotorEncoder();

void InitializeLeftMotorEncoder();
void EnableLeftMotorEncoder();
void DisableLeftMotorEncoder();

char MoveTo(move_t *directions);
void InitializeMotors();



void InitializeMotors(){
    fifo_init(&movements, movementData, MOVEMENT_QUEUE_LENGTH);

    InitializeMotorTimer(800, 1);
    InitializeLeftMotor();
    InitializeRightMotor();

    InitializeLeftMotorEncoder();
    InitializeRightMotorEncoder();

    EnableLeftMotorEncoder();
    EnableRightMotorEncoder();
}

char MoveTo(move_t *directions){
    if (rightRunning || leftRunning){
        return ConvertToSteps(directions);
    }
    else {
        float angle = directions->angle;
        float distance = directions->distance;

        if (angle != 0){
            if (angle < 0){
                rightMotorGoal = round(angle / DEGRESS_PER_STEP);
                leftMotorGoal = 0;
            }
            else {
                leftMotorGoal = round(angle / DEGRESS_PER_STEP);
                rightMotorGoal = 0;
            }
        }

        if (angle != 0 && distance != 0){
            MoveSteps newMovement;
            int steps = round(distance / CM_PER_STEP);
            newMovement.leftSteps = steps;
            newMovement.rightSteps = steps;

            fifo_write(&movements, newMovement);
        }

        if (angle == 0 && distance != 0){
            int steps = round(distance / CM_PER_STEP);
            leftMotorGoal = steps;
            rightMotorGoal = steps;
        }

        if (leftMotorGoal != 0){
            SetDutycycleLeftMotor(PWM_Speed);
        }
        if (rightMotorGoal != 0){
            SetDutycycleRightMotor(PWM_Speed);
        }

    return 1;
    }
}

int ConvertToSteps(move_t *directions){

    int status = 0;
    float angle = directions->angle;
    float distance = directions->distance;

    if (angle != 0){
        MoveSteps newMovement;

        if (angle < 0){
            newMovement.rightSteps = round(angle / DEGRESS_PER_STEP);
            newMovement.leftSteps = 0;
        }
        else {
            newMovement.leftSteps = round(angle / DEGRESS_PER_STEP);
            newMovement.rightSteps = 0;
        }

        status = fifo_write(&movements, newMovement);
    }
    if (distance != 0 && status == 0){
        MoveSteps newMovement;
        int steps = round(distance / CM_PER_STEP);
        newMovement.leftSteps = steps;
        newMovement.rightSteps = steps;

        status = fifo_write(&movements, newMovement);
    }

    return status;
}

void InitializeMotorTimer(int topValue, int prescaler)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    TIM_TimeBaseStructInit(&timerInitStructure);

    timerInitStructure.TIM_Prescaler = prescaler;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    timerInitStructure.TIM_Period = topValue;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);
}

void InitializeLeftMotor()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    GPIO_StructInit(&gpioStructure);

    gpioStructure.GPIO_Pin = GPIO_Pin_3;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);

}

void InitializeRightMotor()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    GPIO_StructInit(&gpioStructure);

    gpioStructure.GPIO_Pin = GPIO_Pin_10;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);
}

void SetDutycycleLeftMotor(int dutycycle)
{
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = dutycycle;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(TIM2, &outputChannelInit);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_1);
}

void SetDutycycleRightMotor(int dutycycle)
{
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = dutycycle;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM2, &outputChannelInit);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);
}

void InitializeRightMotorEncoder()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_4;
    gpioStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);

    EXTI_InitTypeDef extiStructure;
    extiStructure.EXTI_Line = EXTI_Line4;
    extiStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    extiStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    extiStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&extiStructure);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = EXTI4_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    nvicStructure.NVIC_IRQChannelSubPriority = 0x00;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void EnableRightMotorEncoder(){
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void DisableRightMotorEncoder(){
    NVIC_DisableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) { //Check if interrupt flag is set
        EXTI_ClearITPendingBit(EXTI_Line4); //Clear interrupt flag

        if (rightMotorPulseCounter >= rightMotorGoal){
            SetDutycycleRightMotor(0);
            rightMotorPulseCounter = 0;


            if (leftRunning == 0 && fifo_size(&movements) != 0){
                MoveSteps newGoals;
                fifo_read(&movements, &newGoals);
                leftMotorGoal = newGoals.leftSteps;
                rightMotorGoal = newGoals.rightSteps;
                if (leftMotorGoal != 0){
                    SetDutycycleLeftMotor(PWM_Speed);
                }
                if (rightMotorGoal != 0){
                    SetDutycycleRightMotor(PWM_Speed);
                }
            }
            else{
                leftRunning = 0;
            }
        }
        else{

            rightMotorPulseCounter++;
        }
	}
}

void InitializeLeftMotorEncoder()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_5;
    gpioStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);

    EXTI_InitTypeDef extiStructure;
    extiStructure.EXTI_Line = EXTI_Line5;
    extiStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    extiStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    extiStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&extiStructure);

    NVIC_InitTypeDef nvicStructure;

    nvicStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    nvicStructure.NVIC_IRQChannelSubPriority = 0x01;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void EnableLeftMotorEncoder(){
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void DisableLeftMotorEncoder(){
    NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) { //Check if interrupt flag is set
        EXTI_ClearITPendingBit(EXTI_Line5); //Clear interrupt flag

        if (leftMotorPulseCounter >= leftMotorGoal){
            SetDutycycleLeftMotor(0);
            leftMotorPulseCounter = 0;


            if (rightRunning == 0 && fifo_size(&movements) != 0){
                MoveSteps newGoals;
                fifo_read(&movements, &newGoals);
                leftMotorGoal = newGoals.leftSteps;
                rightMotorGoal = newGoals.rightSteps;
                if (leftMotorGoal != 0){
                    SetDutycycleLeftMotor(PWM_Speed);
                }
                if (rightMotorGoal != 0){
                    SetDutycycleRightMotor(PWM_Speed);
                }
            }
            else{
                leftRunning = 0;
            }
        }
        else{


            leftMotorPulseCounter++;
        }

	}
}

