#include "stm32f30x_conf.h"
#include "Uart.h"

/****************************/
/*** USB Serial Functions ***/
/****************************/
void USB_Putchar(char c) {
    USART_SendData(USART2, (char)c);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)  == RESET){}
}

char USB_Getchar() {
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET){};
    char c = (char)USART_ReceiveData(USART2);

    return c;
}

void USB_Putstr(char str[]){
    for(uint16_t i = 0; str[i] != 0; i++){
        USB_Putchar(str[i]);
    }
}

void USB_Init(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= 0x00020000;    // Enable Clock for GPIO Bank A
    RCC->APB1ENR |= 0x00020000;    // Enable Clock for USART2

    // Connect pins to USART2
    GPIOA->AFR[2 >> 0x03] &= ~(0x0000000F << ((2 & 0x00000007) * 4)); // Clear alternate function for PA2
    GPIOA->AFR[2 >> 0x03] |=  (0x00000007 << ((2 & 0x00000007) * 4)); // Set alternate 7 function for PA2
    GPIOA->AFR[3 >> 0x03] &= ~(0x0000000F << ((3 & 0x00000007) * 4)); // Clear alternate function for PA3
    GPIOA->AFR[3 >> 0x03] |=  (0x00000007 << ((3 & 0x00000007) * 4)); // Set alternate 7 function for PA3

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
    USART2->CR1 |= 0x00000001; // Enable USART2
}


/******************************/
/*** UART1 Serial Functions ***/
/******************************/

void UART1_Putchar(char c) {
    USART_SendData(USART1, (char)c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)  == RESET){}
}

char UART1_Getchar() {
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){};
    char c = (char)USART_ReceiveData(USART1);

    return c;
}

void UART1_Init(uint32_t baud) {
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

void UART1_Putstr(char str[]){
    for(uint16_t i = 0; str[i] != 0; i++){
        UART1_Putchar(str[i]);
    }
}

void UART1_send_bytes(uint8_t *data, uint8_t no_bytes)
{
    for(uint8_t i = 0; i < no_bytes; i++){
        USART_SendData(USART1, (uint16_t)data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)  == RESET){}
    }
}


/******************************/
/***      Interrupts        ***/
/******************************/

void UART1_EnableInterrupt(){
    USART1->CR1 |= 0x00000020; //Enable RXNE interrupt (Register not empty)
    NVIC_EnableIRQ(USART1_IRQn); //Enable global interrupt
}



///////////////////////////////////////////////////////////




void USART1_IRQHandler(){

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET){

        uint8_t rx = (uint8_t)USART_ReceiveData(USART1);
        uart_fifo_write(&uart_fifo, &rx);

        //clear flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}




// ---------------- UART FIFO ---------------------- //

//init fifo with buffer
void uart_fifo_init(uart_fifo_t *fifo_in, uint8_t fifo_length, uint8_t *data){
	fifo_in->head = 0;
	fifo_in->tail = 0;
	fifo_in->fifo_size = fifo_length;
	fifo_in->data = data;
}

//read element from fifo
int uart_fifo_read(uart_fifo_t *fifo_in, uint8_t *data_out){

    //check for empty
    if(fifo_in->head == fifo_in->tail){
        return FIFO_EMPTY;
    }

    //retrieve data byte from fido
	*data_out = fifo_in->data[fifo_in->tail];
    fifo_in->tail++;

	//check wrap-around
    if(fifo_in->tail == fifo_in->fifo_size) {
        fifo_in->tail = 0;
    }

    return FIFO_SUCCESS;
}

//write element to fifo
int uart_fifo_write(uart_fifo_t *fifo_in, uint8_t *data_in){

	// Check remaining elements in fifo
	if((fifo_in->head + 1 == fifo_in->tail) || ((fifo_in->head + 1 == fifo_in->fifo_size) && (fifo_in->tail == 0))) {
		return FIFO_FULL;
    }

    // Add data input to fifo
    fifo_in->data[fifo_in->head] = *data_in;
    fifo_in->head++; // increment the head

    //check wrap-around
    if(fifo_in->head == fifo_in->fifo_size) {
        fifo_in->head = 0;
    }

    return FIFO_SUCCESS;
}


//get current number of elements from buffer
int uart_fifo_elements(uart_fifo_t* fifo_in){
    int temp_size = 0;

	if (fifo_in->head > fifo_in->tail){
		temp_size =  (fifo_in->head - fifo_in->tail);
	}
	else if (fifo_in->head < fifo_in->tail){
		temp_size = fifo_in->fifo_size - fifo_in->tail + fifo_in->head;
	}

    return temp_size;
}












