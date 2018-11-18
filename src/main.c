#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "MotorControl.h"

int main(void){

    InitializeMotors();

    init_usb_uart(2000000);

    move_t newMovement;

    newMovement.speed = 1;
    newMovement.angle = 0;
    newMovement.distance = 50;

    MoveTo(&newMovement);


    while(1){
    }

    return 0;
}
