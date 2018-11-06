#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"

#define RX_BUFFER_LENGTH	40			//maximum number of characters to hold in the receive buffer

void UART1_init();
void UART1_IRQHandler();

uint8_t rx_buffer[RX_BUFFER_LENGTH];	//used by the IRQ handler
uint8_t rx_counter = 0; 				//used by the IRQ handler
uint8_t uart_msg[RX_BUFFER_LENGTH];		//variable that contains the latest string received on the RX pin
uint8_t new_uart_msg = 0;				//flag variable to indicate if there is a new message to be serviced

int main(void){




    while(1)
    {

    }

    return 0;
}

void UART1_init()
    {
        USART_InitTypeDef USART_initStructure;
        GPIO_InitTypeDef GPIO_initStructure;
        NVIC_InitTypeDef NVIC_InitStructure;

        //Enable GPIO clock
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        //Enable USART clock
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        //UART configuration
        USART_initStructure.USART_BaudRate              = 115200;
        USART_initStructure.USART_WordLength            = USART_WordLength_8b;
        USART_initStructure.USART_StopBits              = USART_StopBits_1;
        USART_initStructure.USART_Parity                = USART_Parity_No;
        USART_initStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
        USART_initStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(USART1, &USART_initStructure);

        /* Configure USART Tx as alternate function push-pull */
        GPIO_initStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_initStructure.GPIO_Speed = GPIO_Speed_Level_2;
        GPIO_initStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_initStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOA, &GPIO_initStructure);

        /* Configure USART Rx as alternate function push-pull */
        GPIO_initStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOC, &GPIO_initStructure);

        /* Connect PXx to USARTx_Tx */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_5);

        /* Connect PXx to USARTx_Rx */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_5);

        /* Enable USART */
        USART_Cmd(USART1, ENABLE);

        /* Enable the UART1 Receive interrupt: this interrupt is generated when the
        UART4 receive data register is not empty */
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    void UART1_IRQHandler(void)
    {
        int x;

        if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
        {
            /* Read one byte from the receive data register */
            rx_buffer[rx_counter] = USART_ReceiveData(USART1); //(USART_ReceiveData(UART1) & 0x7F);

            /* if the last character received is the NEWLINE ('\n') or LF ('\r') character OR if the RX_BUFFER_LENGTH (40) value has been reached ...*/
            if(rx_counter + 1 == RX_BUFFER_LENGTH || rx_buffer[rx_counter] == '\n' || rx_buffer[rx_counter] == '\r')
            {
                new_uart_msg = 1;
                for(x = 0; x <= rx_counter; x++)
                {//copy each character in the rx_buffer to the uart_msg variable
                    uart_msg[x] = rx_buffer[x];
                }
                uart_msg[x] = '';			//terminate with NULL character
                memset(rx_buffer, 0, RX_BUFFER_LENGTH);		//clear rx_buffer
                rx_counter = 0;
            }
            else
            {
                    rx_counter++;
            }
        }
    }
