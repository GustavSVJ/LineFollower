#include <stm32f30x.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <stdio.h>
#include <string.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_misc.h>
#include <30021_io.h>

#define speed 115200 //Baudrate

void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);

USART_InitTypeDef USART_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;
////void USART_Init(uint32_t speed);
//void Delay(unsigned int);
//void USART1_IRQHandler(void);
//
//uint8_t rx_buffer[RX_BUFFER_LENGTH];	//used by the IRQ handler
//uint8_t rx_counter = 0; 				//used by the IRQ handler
//uint8_t uart_msg[RX_BUFFER_LENGTH];		//variable that contains the latest string received on the RX pin
//uint8_t new_uart_msg = 0;				//flag variable to indicate if there is a new message to be serviced
//
//void USARTx_Init(uint32_t speed)
//{
//	USART_InitTypeDef USART_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	/* Enable GPIO clock */
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//
//	/* Enable USART clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//
//	/* USART configuration */
//	USART_InitStructure.USART_BaudRate = speed;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART1, &USART_InitStructure);
//
//	/* Configure USART Tx as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	/* Configure USART Rx as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	/* Connect PXx to USARTx_Tx */
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_5);
//
//	/* Connect PXx to USARTx_Rx */
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_5);
//
//	/* Enable USART */
//	USART_Cmd(USART1, ENABLE);
//
//	/* Enable the UART4 Receive interrupt: this interrupt is generated when the
//    UART4 receive data register is not empty */
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//
//	/* Enable the USART4 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}
//
//void USARTx_IRQHandler(void)
//{
//    int x;
//
//    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//    {
//        /* Read one byte from the receive data register */
//        rx_buffer[rx_counter] = USART_ReceiveData(USART1); //(USART_ReceiveData(UART4) & 0x7F);
//
//        /* if the last character received is the NEWLINE ('\n') or LF ('\r') character OR if the RX_BUFFER_LENGTH (40) value has been reached ...*/
//        if(rx_counter + 1 == RX_BUFFER_LENGTH || rx_buffer[rx_counter] == '\n' || rx_buffer[rx_counter] == '\r')
//        {
//          new_uart_msg = 1;
//                for(x=0; x<= rx_counter; x++)	//copy each character in the rx_buffer to the uart_msg variable
//                    uart_msg[x] = rx_buffer[x];
//                uart_msg[x] = 0;			//terminate with NULL character
//
//          memset(rx_buffer, 0, RX_BUFFER_LENGTH);		//clear rx_buffer
//          rx_counter = 0;
//        }
//        else
//        {
//            rx_counter++;
//        }
//    }
//}
/*********************************************************************************/


int main(void)
{
    init_usb_uart(9600);

/*************************************************************************/
/********ENABLE CLOCK*****************************************************/
/*************************************************************************/

    //Enable peripheral clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    //Enable GPIO clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    //De-initialize USART
    USART_DeInit(USART1);

/*************************************************************************/
/********SET ALTERNATE FUNCTION #*****************************************/
/*************************************************************************/

    //Connect pin 9 alternate function
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);

    //Connect pin 10 alternate function
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

/*************************************************************************/
/********PIN CONFIGURATION TX*********************************************/
/*************************************************************************/

    //Connect the pin to the desired peripheral
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;

    //Set alternate function
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;

    //Set type push/pull
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

    //Set pull up
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    //Set speed
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    //Call init function
    GPIO_Init(GPIOA, &GPIO_InitStruct);

/*************************************************************************/
/********PIN CONFIGURATION RX*********************************************/
/*************************************************************************/

    //Connect the pin to the desired peripheral
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;

    //Set alternate function
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;

    //Set type Push/pull
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

    //Set pull up
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    //Set speed
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    //Call init function
    GPIO_Init(GPIOA, &GPIO_InitStruct);

/********************************************************************/
/********USART VALUES************************************************/
/********************************************************************/

    USART_InitStruct.USART_BaudRate = speed;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStruct);

/********************************************************************/
/********INITIALIZE USART********************************************/
/********************************************************************/

    USART_Init(USART1, &USART_InitStruct);

/********************************************************************/
/********************************************************************/
/********************************************************************/

    while(1)
    {
        printf("test");
        USART_SendData(USART1, 0x55);
        unsigned char byte = USART_ReceiveData(USART1);

        asm ("nop");
    }

    return 0;
}
