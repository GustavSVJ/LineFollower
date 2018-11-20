#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "flash.h"
#include <stdio.h>
#include <string.h>

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
void uart3_putc(uint8_t c) {
    USART_SendData(USART3, (uint8_t)c);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)  == RESET){}
}

uint8_t uart3_getc() {
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET){};
    uint8_t c = (uint8_t)USART_ReceiveData(USART3);

    if (c != 0x0D) { uart3_putc(c); }

    return c;
}

void uart3_init(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= RCC_AHBPeriph_GPIOB;    // Enable Clock for GPIO Bank A
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;   // Enable Clock for USART2

    // Connect pins to USART2
    GPIOB->AFR[10 >> 0x03] &= ~(0x0000000F << ((10 & 0x00000007) * 4)); // Clear alternate function for PA2
    GPIOB->AFR[10 >> 0x03] |=  (0x00000007 << ((10 & 0x00000007) * 4)); // Set alternate function 7 (USART2) for PA2
    GPIOB->AFR[11 >> 0x03] &= ~(0x0000000F << ((11 & 0x00000007) * 4)); // Clear alternate function for PA3
    GPIOB->AFR[11 >> 0x03] |=  (0x00000007 << ((11 & 0x00000007) * 4)); // Set alternate function 7 (USART2) for PA3

    // Configure pins PA2 and PA3 for 10 MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (10 * 2) | 0x00000003 << (11 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (10 * 2) | 0x00000001 << (11 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (10)     | 0x0001     << (11));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (10)     | 0x0000     << (11));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (10 * 2) | 0x00000003 << (11 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (10 * 2) | 0x00000002 << (11 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (10 * 2) | 0x00000003 << (11 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000001 << (10 * 2) | 0x00000001 << (11 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //Configure USART2
    USART3->CR1 &= ~0x00000001; // Disable USART2
    USART3->CR2 &= ~0x00003000; // Clear CR2 Configuration
    USART3->CR2 |=  0x00000000; // Set 1 stop bits
    USART3->CR1 &= ~(0x00001000 | 0x00000400 | 0x00000200 | 0x00000008 | 0x00000004); // Clear CR1 Configuration
    USART3->CR1 |=  0x00000000; // Set word length to 8 bits
    USART3->CR1 |=  0x00000000; // Set parity bits to none
    USART3->CR1 |=  0x00000004 | 0x00000008; // Set mode to RX and TX
    USART3->CR3 &= ~(0x00000100 | 0x00000200); // Clear CR3 Configuration
    USART3->CR3 |=  0x00000000; // Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART2 Clock frequency
    apbclock = RCC_ClocksStatus.USART3CLK_Frequency;

    if ((USART3->CR1 & 0x00008000) != 0) {
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

    if ((USART3->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    USART3->BRR = (uint16_t)divider; // Configure baud rate
    USART3->CR1 |= 0x00000001; // Enable USART1
}

void uart3_putstr(uint8_t str[]){
    for(uint8_t i = 0; str[i] != 0; i++){
        uart3_putc(str[i]);
    }

}

/******************************/
/***      Interrupts        ***/
/******************************/

void init_USART1interrupt(){
    USART1->CR1 |= 0x00000020; //Enable RXNE interrupt (Register not empty)
    NVIC_EnableIRQ(USART1_IRQn);
}

/*********************************************************************/
int main(void){

    uart1_init(115200);
    uart3_init(115200);

//    uint8_t str[8];
//    memset(str, 0, 8);
//    sprintf(str, 0x55);
//    fflush(stdout);
    uint8_t str1[] = {0x54, 0x45};
    uint8_t str2[] = {0x53, 0x54};

    while(1){
        uart1_putstr(str1);
        //Delay
        for (uint32_t i = 0; i < 0xfffff; i++);
        uart3_putstr(str2);
        //Delay
        for (uint32_t i = 0; i < 0xfffff; i++);
    }

    return 0;
}
