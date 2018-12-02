#include "stm32f30x.h"
#include "Uart.h"
#include "UcamFunction.h"


ucam_status ucam_init(void){

    //temp status
    ucam_status status = UCAM_SUCCESS;

    //setup reset pin for ucam
    ucam_reset_pin_setup();

    //hardware reset ucam
    ucam_reset();

    //sync with ucam
    status = ucam_sync();

    //setup image format
    status = ucam_im_setup();

    //delay time needed before get picture is allowed
    delay_ms(1000);

    return status;
}


ucam_status ucam_get_picture(uint8_t * im){



    // GET PICTURE
    uint8_t setup_data_2[6] = {0xAA, 0x04, 0x02, 0x00, 0x00, 0x00};

    uint8_t ack_data[6]   = {0xAA, 0x0E, 0x04, 0x00, 0x00, 0x00};

    uint8_t fifo_data;
    ucam_status status = UCAM_SUCCESS;



    //sync with ucam
    status = ucam_sync();


    UART1_send_bytes(setup_data_2, 6);

    while(uart_fifo_elements(&uart_fifo) < 6);  //wait for ACK/NAK

    for(int i = 0; i < sizeof(ack_data); i++){  //check ack package from cam

        uart_fifo_read(&uart_fifo, &fifo_data); //read element from fifo

        if(i != 3){                             //compare to expected values
            if(fifo_data != ack_data[i])
                status = UCAM_ACK_FAIL;
        }
    }






    uint8_t im_data[6]   = {0xAA, 0x0A, 0x02, 0xC0, 0x12, 0x00};

    while(uart_fifo_elements(&uart_fifo) < 6);

    for(int i = 0; i < sizeof(im_data); i++){  //check ack package from cam

        uart_fifo_read(&uart_fifo, &fifo_data); //read element from fifo

        if(fifo_data != im_data[i])
            status = UCAM_ACK_FAIL;
    }

    for(int i = 0; i < 4800; i++){

        while(uart_fifo_elements(&uart_fifo) < 1);

        uart_fifo_read(&uart_fifo, &fifo_data);
        *im = fifo_data;
        im++;
    }

    ack_data[2] = 0x0A;
    UART1_send_bytes(ack_data, 6);

    return status;
}


ucam_status ucam_im_setup(void){

    // INITIAL register (AA01)
    // image format     - 0x03 - 8-bit grey
    // Raw resolution   - 0x01 - 80x60
    // JPEG Resolution  - 0x03 - don't care

    uint8_t setup_data[6] = {0xAA, 0x01, 0x00, 0x03, 0x01, 0x03};
    uint8_t ack_data[6]   = {0xAA, 0x0E, 0x01, 0x00, 0x00, 0x00};
    uint8_t fifo_data;
    ucam_status status = UCAM_SUCCESS;

    UART1_send_bytes(setup_data, 6);            //send setup command
    while(uart_fifo_elements(&uart_fifo) < 6);  //wait for ACK/NAK

    for(int i = 0; i < sizeof(ack_data); i++){  //check ack package from cam

        uart_fifo_read(&uart_fifo, &fifo_data); //read element from fifo

        if(i != 3){                             //compare to expected values
            if(fifo_data != ack_data[i])
                status = UCAM_ACK_FAIL;
        }
    }

    return status;
}



ucam_status ucam_sync(void){

    uint8_t sync_data[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};    //sync package to/from cam
    uint8_t ack_data[6]  = {0xAA, 0x0E, 0x0D, 0xDC, 0x00, 0x00};    //ack package to/from cam

    ucam_status status = UCAM_SYNC_FAIL;                            //variables
    uint8_t sync_tries = 60;
    uint8_t fifo_data;

    while(status == UCAM_SYNC_FAIL && sync_tries != 0){             //sync process, max 60 tries

        while(uart_fifo_read(&uart_fifo, &fifo_data) != FIFO_EMPTY);//empty uart fifo if data is present
        UART1_send_bytes(sync_data, 6);                             //send sync command
        delay_100us(3);                                                //wait for response

        if(uart_fifo_elements(&uart_fifo) >= 12){                   //check for response

            ucam_status status_temp = UCAM_SUCCESS;                 //temp status

            for(int i = 0; i < sizeof(ack_data); i++){              //check ack package from cam

                uart_fifo_read(&uart_fifo, &fifo_data);             //read element from fifo

                if(i == 3){                                         //compare to expected values
                    ack_data[i] = fifo_data;
                }
                else{
                    if(fifo_data != ack_data[i])
                        status_temp = UCAM_ACK_FAIL;                //change temp status if wrong data
                }
            }

            for(int i = 0; i < sizeof(sync_data); i++){             //check sync package from cam

                uart_fifo_read(&uart_fifo, &fifo_data);             //read element from fifo

                if(fifo_data != sync_data[i]){                      //compare to expected values
                    status_temp = UCAM_ACK_FAIL;                    //change temp status if wrong data
                }

            }

            if(status_temp == UCAM_SUCCESS){                        //packages ok check

                UART1_send_bytes(ack_data, 6);                      //send ack for cam sync
                return status_temp;                                 //update function status
            }
        }
        delay_ms(4 + (60 - sync_tries));                            //wait before new sync request
        sync_tries--;                                               //count down number of sync tries
    }
    return status;  //return final status
} //end ucam_sync





// ************************************ //
//  Ucam3 hardware reset functions      //
// ------------------------------------ //

void ucam_reset_pin_setup(void){

    //setup PC08 as output for ucam reset pin
    //reset is active low

    RCC->AHBENR |= RCC_AHBPeriph_GPIOC;         //Enable clock for GPIO Port C

    GPIOC->MODER &= ~(0x00000003 << (8 * 2));   //clear MODER for PC08
    GPIOC->MODER |=  (0x00000001 << (8 * 2));   //set MODER for output

    GPIOC->PUPDR &= ~(0x00000003 << (0 * 2));   //clear PUPDR for PC08
    GPIOC->PUPDR |=  (0x00000001 << (0 * 2));   //set PUPDR for pull-up

    GPIOC->ODR |= (0x0001 << 8);                //set PCO8 as high
}


void ucam_reset(void){

    //reset ucam3 with PC08, reset is active low
    GPIOC->ODR &= ~(0x0001 << 8);

    //wait for 10ms
    delay_ms(10);

    //activate camera again
    GPIOC->ODR |= (0x0001 << 8);

    //wait for 1ms
    delay_ms(1);
}




// ************************************ //
//  Software delay functions            //
// ------------------------------------ //

void delay_ms(uint32_t time_ms){

    //update SysTick delay timer
    delay_timer = time_ms*10;

    //wait until time_ms has passed
    while(delay_timer != 0);
}


void delay_100us(uint32_t time_100us){

    //update SysTick delay timer
    SysTick->VAL   = 0;
    delay_timer = time_100us;

    //wait until time_ms has passed
    while(delay_timer != 0);
}

void init_delay_timer(void){
    //setup systick to be used for ms delays
    SysTick_Config(64000000/10000);
}


void SysTick_Handler(void){
    //check if delay timer is set, and count down if it is
    if(delay_timer > 0){
        delay_timer--;
    }
}
