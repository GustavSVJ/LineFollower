#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "MotorControl.h"

int main(void){

    InitializeMotors();

    init_usb_uart(115200);

    move_t newMovement;

    newMovement.speed = 1;
    newMovement.angle = 0;
    newMovement.distance = 200;

    MoveTo(&newMovement);


    while(1){
        for(uint16_t i = 'a'; i < 'z' + 1; i++){
            uart_putc(i);
        for(uint32_t j = 0; j < 0xfffff; j++);
        }

    }

    return 0;
}
