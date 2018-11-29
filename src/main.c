#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "MotorControl.h"

int main(void){

    InitializeMotors();

    init_usb_uart(115200);

    move_t newMovement;



    while(1){
            newMovement.speed = 1;
            newMovement.angle = 0;
            newMovement.distance = 200;

            MoveTo(&newMovement);
        uart_putc('o');
        for(uint64_t j = 0; j < 0xffffff; j++);
        uart_putc('n');


    }

    return 0;
}
