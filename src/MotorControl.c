#include "MotorControl.h"
#include "stm32f30x_conf.h" // STM32 config

void InitializeMotorTimer(int topValue, int prescaler);
void InitializeLeftMotor();
void InitializeRightMotor();

void InitializeRightMotorEncoder();
void InitializeLeftMotorEncoder();

void EnableRightMotorEncoder();
void DisableRightMotorEncoder();

void EnableLeftMotorEncoder();
void DisableLeftMotorEncoder();

void InitializeMotors(uint16_t top, uint16_t prescaler){

    InitializeMotorTimer(top, prescaler);
    InitializeLeftMotor();
    InitializeRightMotor();

    InitializeLeftMotorEncoder();
    InitializeRightMotorEncoder();

    EnableLeftMotorEncoder();
    EnableRightMotorEncoder();
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
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpioStructure);

}

void InitializeRightMotor()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    GPIO_StructInit(&gpioStructure);

    gpioStructure.GPIO_Pin = GPIO_Pin_10;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpioStructure);
}

void SetDutycycleLeftMotor(int dutycycle)
{
    TIM_OCInitTypeDef outputChannelInit;
    TIM_OCStructInit(&outputChannelInit);

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
    TIM_OCInitTypeDef outputChannelInit;
    TIM_OCStructInit(&outputChannelInit);

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
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;
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

void EnableRightMotorEncoder()
{
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void DisableRightMotorEncoder()
{
    NVIC_DisableIRQ(EXTI4_IRQn);
}


void InitializeLeftMotorEncoder()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_5;
    gpioStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioStructure.GPIO_Speed = GPIO_Speed_2MHz;
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
    nvicStructure.NVIC_IRQChannelSubPriority = 0x00;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void EnableLeftMotorEncoder(){
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void DisableLeftMotorEncoder(){
    NVIC_DisableIRQ(EXTI9_5_IRQn);
}


