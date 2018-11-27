#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "flash.h"
#include <stdio.h>
#include <string.h>
#include "30021_io.h"

/******************************/
/*** UART1 Serial Functions ***/
/******************************/


void uart1_putc(uint8_t c) {
    USART_SendData(USART1, (uint8_t)c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)  == RESET){}
}

uint8_t uart1_getc() {
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){};
    uint8_t c = (uint8_t)USART_ReceiveData(USART1);

    if (c != 0x0D) { uart1_putc(c); }

    return c;
}

void uart1_init(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= RCC_AHBPeriph_GPIOC;    // Enable Clock for GPIO Bank C
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // Enable Clock for USART1

    // Connect pins to USART1
    GPIOC->AFR[5 >> 0x03] &= ~(0x0000000F << ((5 & 0x00000007) * 4)); // Clear alternate function for PC5
    GPIOC->AFR[5 >> 0x03] |=  (0x00000007 << ((5 & 0x00000007) * 4)); // Set alternate function 7 (USART1) for PC5
    GPIOC->AFR[4 >> 0x03] &= ~(0x0000000F << ((4 & 0x00000007) * 4)); // Clear alternate function for PC4
    GPIOC->AFR[4 >> 0x03] |=  (0x00000007 << ((4 & 0x00000007) * 4)); // Set alternate function 7 (USART1) for PC4

    // Configure pins PC5 and PC4 for 10 MHz alternate function
    GPIOC->OSPEEDR &= ~(0x00000003 << (5 * 2) | 0x00000003 << (4 * 2));    // Clear speed register
    GPIOC->OSPEEDR |=  (0x00000001 << (5 * 2) | 0x00000001 << (4 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOC->OTYPER  &= ~(0x0001     << (5)     | 0x0001     << (4));        // Clear output type register
    GPIOC->OTYPER  |=  (0x0000     << (5)     | 0x0000     << (4));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOC->MODER   &= ~(0x00000003 << (5 * 2) | 0x00000003 << (4 * 2));    // Clear mode register
    GPIOC->MODER   |=  (0x00000002 << (5 * 2) | 0x00000002 << (4 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOC->PUPDR   &= ~(0x00000003 << (5 * 2) | 0x00000003 << (4 * 2));    // Clear push/pull register
    GPIOC->PUPDR   |=  (0x00000001 << (5 * 2) | 0x00000001 << (4 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //Configure USART1
    USART1->CR1 &= ~0x00000001; // Disable USART1
    USART1->CR2 &= ~0x00003000; // Clear CR2 Configuration
    USART1->CR2 |=  0x00000000; // Set 1 stop bits
    USART1->CR1 &= ~(0x00001000 | 0x00000400 | 0x00000200 | 0x00000008 | 0x00000004); // Clear CR1 Configuration
    USART1->CR1 |=  0x00000000; // Set word length to 8 bits
    USART1->CR1 |=  0x00000000; // Set parity bits to none
    USART1->CR1 |=  0x00000004 | 0x00000008; // Set mode to RX and TX
    USART1->CR3 &= ~(0x00000100 | 0x00000200); // Clear CR3 Configuration
    USART1->CR3 |=  0x00000000; // Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART1 Clock frequency
    apbclock = RCC_ClocksStatus.USART2CLK_Frequency;

    if ((USART1->CR1 & 0x00008000) != 0) {
      // (divider * 10) computing in case Oversampling mode is 8 Samples
      divider = (2 * apbclock) / baud;
      tmpreg  = (2 * apbclock) % baud;
    } else {
      // (divider * 10) computing in case Oversampling mode is 16 Samples
      divider = apbclock / baud;
      tmpreg  = apbclock % baud;
    }

    if (tmpreg >=  baud / 2) {
        divider++;
    }

    if ((USART1->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    USART1->BRR = (uint16_t)divider; // Configure baud rate
    USART1->CR1 |= 0x00000001; // Enable USART1
}

void uart1_putstr(uint8_t str[]){
    for(uint8_t i = 0; str[i] != 0; i++){
        uart1_putc(str[i]);
    }

}
/******************************/
/*** UART2 Serial Functions ***/
/******************************/
void uart2_putc(uint8_t c) {
    USART_SendData(USART2, (uint8_t)c);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)  == RESET){}
}

uint8_t uart2_getc() {
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET){};
    uint8_t c = (uint8_t)USART_ReceiveData(USART2);

    if (c != 0x0D) { uart2_putc(c); }

    return c;
}

void uart2_init(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= RCC_AHBPeriph_GPIOA;    // Enable Clock for GPIO Bank A
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable Clock for USART2

    // Connect pins to USART2
    GPIOA->AFR[2 >> 0x03] &= ~(0x0000000F << ((2 & 0x00000007) * 4)); // Clear alternate function for PA2
    GPIOA->AFR[2 >> 0x03] |=  (0x00000007 << ((2 & 0x00000007) * 4)); // Set alternate function 7 (USART2) for PA2
    GPIOA->AFR[3 >> 0x03] &= ~(0x0000000F << ((3 & 0x00000007) * 4)); // Clear alternate function for PA3
    GPIOA->AFR[3 >> 0x03] |=  (0x00000007 << ((3 & 0x00000007) * 4)); // Set alternate function 7 (USART2) for PA3

    // Configure pins PA2 and PA3 for 10 MHz alternate function
    GPIOA->OSPEEDR &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // Clear speed register
    GPIOA->OSPEEDR |=  (0x00000001 << (2 * 2) | 0x00000001 << (3 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~(0x0001     << (2)     | 0x0001     << (3));        // Clear output type register
    GPIOA->OTYPER  |=  (0x0000     << (2)     | 0x0000     << (3));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOA->MODER   &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // Clear mode register
    GPIOA->MODER   |=  (0x00000002 << (2 * 2) | 0x00000002 << (3 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR   &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // Clear push/pull register
    GPIOA->PUPDR   |=  (0x00000001 << (2 * 2) | 0x00000001 << (3 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //Configure USART2
    USART2->CR1 &= ~0x00000001; // Disable USART2
    USART2->CR2 &= ~0x00003000; // Clear CR2 Configuration
    USART2->CR2 |=  0x00000000; // Set 1 stop bits
    USART2->CR1 &= ~(0x00001000 | 0x00000400 | 0x00000200 | 0x00000008 | 0x00000004); // Clear CR1 Configuration
    USART2->CR1 |=  0x00000000; // Set word length to 8 bits
    USART2->CR1 |=  0x00000000; // Set parity bits to none
    USART2->CR1 |=  0x00000004 | 0x00000008; // Set mode to RX and TX
    USART2->CR3 &= ~(0x00000100 | 0x00000200); // Clear CR3 Configuration
    USART2->CR3 |=  0x00000000; // Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART2 Clock frequency
    apbclock = RCC_ClocksStatus.USART2CLK_Frequency;

    if ((USART2->CR1 & 0x00008000) != 0) {
      // (divider * 10) computing in case Oversampling mode is 8 Samples
      divider = (2 * apbclock) / baud;
      tmpreg  = (2 * apbclock) % baud;
    } else {
      // (divider * 10) computing in case Oversampling mode is 16 Samples
      divider = apbclock / baud;
      tmpreg  = apbclock % baud;
    }

    if (tmpreg >=  baud / 2) {
        divider++;
    }

    if ((USART2->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    USART2->BRR = (uint16_t)divider; // Configure baud rate
    USART2->CR1 |= 0x00000001; // Enable USART1
}

void uart2_putstr(uint8_t str[]){
    for(uint8_t i = 0; str[i] != 0; i++){
        uart1_putc(str[i]);
    }

}

/******************************/
/***      Interrupts        ***/
/******************************/

void init_USART1interrupt(){
    USART1->CR1 |= 0x00000020; //Enable RXNE interrupt (Register not empty)
    NVIC_EnableIRQ(USART1_IRQn); //Enable global interrupt
}

/*********************************************************************/
int main(void){
    init_usb_uart(921600);
    uart1_init(921600);
    init_USART1interrupt();

    //Initializing camera:
    uint8_t init_camera[] = {0xAA,0x01,0x00,0x03,0x01,0x00};
    uart1_putstr(init_camera);

    //Sync camera
    uint8_t sync_camera[] = {0xAA,0x0D,0x00,0x00,0x00,0x00};
//    uart1_putstr(sync_camera);

    while(1){
        if(USART_ReceiveData(USART1) != sync_camera){
            uart1_putstr(sync_camera);
        }
        //Delay
        for (uint32_t i = 0; i < 0xfffff; i++);
    }

    return 0;
}

void USART1_IRQHandler(){
    FlagStatus stuff = USART_GetFlagStatus(USART1, USART_FLAG_RXNE);
    if (stuff != RESET){
//        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        uart_putc(USART_ReceiveData(USART1));
    }
}
