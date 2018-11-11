#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "MotorControl.h"

int main(void){
    InitializeLeftMotor();

    InitializeMotorTimer(800, 1);

    init_usb_uart(2000000);

    InitializeLeftMotorEncoder();
    EnableLeftMotorEncoder();

    SetDutycycleLeftMotor(300);



    while(1){
    }

    return 0;
}
