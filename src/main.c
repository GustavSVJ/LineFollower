#include "stm32f30x_conf.h"
#include "stm32f30x.h"

#include <stdio.h>
#include <string.h>
#include "stm32f30x_conf.h"

#include "Uart.h"
#include "MotorControl.h"
#include "PID.h"

char uart1_ReceiveBuffer[100];
volatile char uart1_RxFlag = 0;

int main(void){

    USB_Init(921600);

    RegulatorRun();

    /*

    RegulatorSetRefs(40,40,0);

    uint16_t i = 0;

    uart_fifo_init(&uart_fifo, sizeof(uart_data_buffer), uart_data_buffer);

    UART1_Init(921600);
    UART1_EnableInterrupt();

    uint8_t tester[6] = {72, 69, 74, 60, 51, 31};
    UART1_send_bytes(tester, 6);

    ucam_init();

    */

    while(1){

        DriveTo(5000,0,40);
        DriveTo(5000,90,40);
        DriveTo(5000,-90,40);
        break;
    }

    return 0;

} //end main
