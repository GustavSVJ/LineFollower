#include <stm32f30x.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <stdio.h>
#include <string.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_misc.h>

#define RX_BUFFER_LENGTH	40			//maximum number of characters to hold in the receive buffer

void UART4_Init(uint32_t speed);
void Delay(unsigned int);
void UART4_IRQHandler(void);

uint8_t rx_buffer[RX_BUFFER_LENGTH];	//used by the IRQ handler
uint8_t rx_counter = 0; 				//used by the IRQ handler
uint8_t uart_msg[RX_BUFFER_LENGTH];		//variable that contains the latest string received on the RX pin
uint8_t new_uart_msg = 0;				//flag variable to indicate if there is a new message to be serviced

void UART4_Init(uint32_t speed){

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_5);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_5);

	/* Enable USART */
	USART_Cmd(UART4, ENABLE);

	/* Enable the UART4 Receive interrupt: this interrupt is generated when the
    UART4 receive data register is not empty */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	/* Enable the USART4 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void UART4_IRQHandler(void)
{
    int x;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
        rx_buffer[rx_counter] = USART_ReceiveData(UART4); //(USART_ReceiveData(UART4) & 0x7F);

        /* if the last character received is the NEWLINE ('\n') or LF ('\r') character OR if the RX_BUFFER_LENGTH (40) value has been reached ...*/
        if(rx_counter + 1 == RX_BUFFER_LENGTH || rx_buffer[rx_counter] == '\n' || rx_buffer[rx_counter] == '\r')
        {
          new_uart_msg = 1;
                for(x=0; x<= rx_counter; x++)	//copy each character in the rx_buffer to the uart_msg variable
                    uart_msg[x] = rx_buffer[x];
                uart_msg[x] = 0;			//terminate with NULL character

          memset(rx_buffer, 0, RX_BUFFER_LENGTH);		//clear rx_buffer
          rx_counter = 0;
        }
        else
        {
            rx_counter++;
        }
    }
}

void uart_putc(uint8_t c) {
    USART_SendData(UART4, (uint8_t)c);
    while(USART_GetFlagStatus(UART4, USART_FLAG_TXE)  == RESET){}
}

uint8_t uart_getc() {
    while(USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET){};
    uint8_t c = (uint8_t)USART_ReceiveData(UART4);

    if (c != 0x0D) { uart_putc(c); }

    return c;
}

int main(void)
{
	UART4_Init(115200);		//initialize the UART4 module at 115200 baud

    while(1)
    {

    }

    return 0;


}
