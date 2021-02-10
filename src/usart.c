/**
  ******************************************************************************
  * @file    usart.c
  * @author  Ac6
  * @version V1.0
  * @date    25-March-2019
  * @brief   USART Configuration Functions.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include "string.h"
#include <GPS.h>


void usart1_init(int);
void dma_usart1_init(void);
void send_usart1(char *);

void usart2_init(int);
void dma_usart2_init(void);
void send_usart2(char *);

int usart1_rx_index = 0;                  //index for receive buffer
int usart2_rx_index = 0;                  //index for receive buffer
extern char usart1_rx_message[16];   //buffer
extern char usart2_rx_message[MAXLINELENGTH];   //buffer
int usart1_rx_message_counter = 0;     //counter of 0xff for touch screen message
extern int usart1_rx_complete_flag;   //flag for a complete touch screen message
extern int usart2_rx_complete_flag;   //flag for a complete gps message


void usart1_init(int baud)
{
    //ENABLE GPIO
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(3<<(9*2));        //TX
    GPIOA->MODER |= 2<<(9*2);

    GPIOA->MODER &= ~(3<<(10*2));       //RX
    GPIOA->MODER |= 2<<(10*2);

    GPIOA->AFR[1] &= ~(0xf << (4*(9-8)));       //alternate function for PA9 for usart1
    GPIOA->AFR[1] |= 0x1 << (4*(9-8));

    GPIOA->AFR[1] &= ~(0xf << (4*(10-8)));       //alternate function for PA10 for usart1
    GPIOA->AFR[1] |= 0x1 << (4*(10-8));

    //ENABLE USART
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR1 &= ~USART_CR1_UE;       //DISABLE
    USART1->CR1 &= ~USART_CR1_M;        //WORD LENGTH TO BE 8 BITS 1 START, 1 STOP, NO PARITY
    USART1->CR1 &= ~(USART_CR1_M << 16);//CLEARING BOTH M BITS
    USART1->CR1 &= ~USART_CR1_OVER8;    //OVERSAMPLE BY 16
    USART1->CR1 |= USART_CR1_RXNEIE;    //ENABLE RECIEVER INTERRUPT
    USART1->CR1 |= USART_CR1_TE;        //ENABLE TX
    USART1->CR1 |= USART_CR1_RE;        //ENABLE RX

    USART1->CR2 |= USART_CR2_RTOEN;     //ENABLE RECEIVER TIMEOUT
    USART1->CR2 &= ~USART_CR2_CPOL;     //CLOCK POLARITY
    USART1->CR2 &= ~USART_CR2_CPHA;     //CLOCK PHASE

    USART1->BRR = (48000000+1)/(baud-1);  //DIVISOR FOR 9600 BAUD

    //USART1->RTOR |= USART_RTOR_RTO;     //RECIEVER TIMEOUT

    USART1->CR1 |= USART_CR1_UE;        //ENABLE

    NVIC->ISER[0] = 1<<USART1_IRQn;     //ENABLE INTERRUPTS FOR USART 1
}

void dma_usart1_init()
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;       //ENABLE DMA CLOCK

    //DMA USART1 TX
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;      //DISABLE TO MAKE CHANGES
    DMA1_Channel2->CCR |= DMA_CCR_DIR;      //READ FROM MEMORY
    DMA1_Channel2->CCR &= ~DMA_CCR_MSIZE;   //RESET DATA SIZE TO 8 BITS
    DMA1_Channel2->CCR &= ~DMA_CCR_PSIZE;   //RESET DATA SIZE TO 8 BITS
    DMA1_Channel2->CCR |= DMA_CCR_MINC;     //INCREMENT MEMORY
    DMA1_Channel2->CCR &= ~DMA_CCR_PINC;    //DON'T INCREMENT PERIPHERAL
    DMA1_Channel2->CCR &= ~DMA_CCR_CIRC;    //DISABLE CIRCULAR MODE
    DMA1_Channel2->CCR &= ~DMA_CCR_MEM2MEM; //MEMORY TO PERIPHERAL
    DMA1_Channel2->CCR &= ~DMA_CCR_PL;      //LOW PRIORITY

    DMA1_Channel2->CCR |= DMA_CCR_TEIE | DMA_CCR_TCIE;  //ENABLE INTERRUPTS

    USART1->CR3 |= USART_CR3_DMAT;          //ENABLE DMA FOR TRANSMIT

    NVIC->ISER[0] = 1<<DMA1_Channel2_3_IRQn;  //ENABLE INTERRUPTS FOR CHANNELS 2 & 3
}

void DMA1_Channel2_3_IRQHandler(void)
{

    //CHANNEL 2
    if (DMA1->ISR & DMA_ISR_GIF2)
        DMA1->IFCR |= DMA_IFCR_CGIF2;       //CLEAR GLOBAL INTERRUPT FLAG

    if (DMA1->ISR & DMA_ISR_TCIF2)
        DMA1->IFCR |= DMA_IFCR_CTCIF2;      //CLEAR TRANSFER COMPLETE INTERRUPT FLAG

    if (DMA1->ISR & DMA_ISR_HTIF2)
        DMA1->IFCR |= DMA_IFCR_CHTIF2;       //CLEAR HALF TRANSFER COMPLETE INTERRUPT FLAG

    if (DMA1->ISR & DMA_ISR_TEIF2)
        DMA1->IFCR |= DMA_IFCR_CTEIF2;       //CLEAR TRANSFER ERROR INTERRUPT FLAG

}

void USART1_IRQHandler()
{
    if (USART1->ISR & USART_ISR_RXNE)      //RX REGISTER NOT EMPTY
    {
        char chartoreceive = (uint8_t)(USART1->RDR);
        usart1_rx_message[usart1_rx_index] = chartoreceive;
        usart1_rx_index++;
        if (chartoreceive == 0xff)
            usart1_rx_message_counter++;           //increment the touch screen counter
        if (usart1_rx_message_counter >= 3)        //three 0xff in a row indicates a complete message
        {
            usart1_rx_index = 0;
            usart1_rx_message_counter = 0;
            usart1_rx_complete_flag = 1;
        }

        USART1->RQR |= USART_RQR_RXFRQ;     //CLEAR INTERRUPT
    }

    if(USART1->ISR & USART_ISR_ORE)
        USART1->ICR |= USART_ICR_ORECF;     //CLEAR OVERRUN INTERRUPT

}


void send_usart1(char *usart1_tx_dma_buffer)
{
    uint32_t timeout = -1;

    while(DMA1->ISR & DMA_ISR_TCIF2);           //WAIT FOR TRANSFER COMPLETE FLAG TO BE CLEARED
    while(!(USART1->ISR & USART_ISR_TC))        //WAIT FOR TRANSMISSION COMPLETE
    {
        timeout++;
        if(timeout > 10000)
            return;
    }
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CMAR = (uint32_t) usart1_tx_dma_buffer;
    DMA1_Channel2->CPAR = (uint32_t) &(USART1->TDR);
    DMA1_Channel2->CNDTR = strlen(usart1_tx_dma_buffer);
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    timeout = 0;
    while(!(USART1->ISR & USART_ISR_TC))        //WAIT FOR TRANSMISSION COMPLETE
    {
        timeout++;
        if(timeout > 10000)
            return;
    }
    return;
}

/*USART 2 FUNCTIONS*/

void usart2_init(int baud)
{
    //ENABLE GPIO
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(3<<(2*2));        //TX
    GPIOA->MODER |= 2<<(2*2);

    GPIOA->MODER &= ~(3<<(3*2));       //RX
    GPIOA->MODER |= 2<<(3*2);

    GPIOA->AFR[1] &= ~(0xf << (4*2));       //alternate function for PA2 for usart2
    GPIOA->AFR[1] |= 0x1 << (4*2);

    GPIOA->AFR[1] &= ~(0xf << (4*3));       //alternate function for PA3 for usart2
    GPIOA->AFR[1] |= 0x1 << (4*3);

    //ENABLE USART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 &= ~USART_CR1_UE;       //DISABLE
    USART2->CR1 &= ~USART_CR1_M;        //WORD LENGTH TO BE 8 BITS 1 START, 1 STOP, NO PARITY
    USART2->CR1 &= ~(USART_CR1_M << 16);//CLEARING BOTH M BITS
    USART2->CR1 |= USART_CR1_RTOIE;     //ENABLE RECEIVER TIMEOUT INTERRUPT
    USART2->CR1 &= ~USART_CR1_OVER8;    //OVERSAMPLE BY 16
    USART2->CR1 |= USART_CR1_RXNEIE;    //ENABLE RECIEVER INTERRUPT
    USART2->CR1 |= USART_CR1_TE;        //ENABLE TX
    USART2->CR1 |= USART_CR1_RE;        //ENABLE RX

    USART2->CR2 |= USART_CR2_RTOEN;     //ENABLE RECEIVER TIMEOUT
    USART2->CR2 &= ~USART_CR2_CPOL;     //CLOCK POLARITY
    USART2->CR2 &= ~USART_CR2_CPHA;     //CLOCK PHASE

    USART2->BRR = (48000000+1)/(baud-1);  //DIVISOR FOR 9600 BAUD

    USART2->RTOR |= USART_RTOR_RTO & 2;     //RECIEVER TIMEOUT

    USART2->CR1 |= USART_CR1_UE;        //ENABLE

    NVIC->ISER[0] = 1<<USART2_IRQn;     //ENABLE INTERRUPTS FOR USART 2
}

void dma_usart2_init()
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;       //ENABLE DMA CLOCK

    //DMA USART2 TX
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;      //DISABLE TO MAKE CHANGES
    DMA1_Channel4->CCR |= DMA_CCR_DIR;      //READ FROM MEMORY
    DMA1_Channel4->CCR &= ~DMA_CCR_MSIZE;   //RESET DATA SIZE TO 8 BITS
    DMA1_Channel4->CCR &= ~DMA_CCR_PSIZE;   //RESET DATA SIZE TO 8 BITS
    DMA1_Channel4->CCR |= DMA_CCR_MINC;     //INCREMENT MEMORY
    DMA1_Channel4->CCR &= ~DMA_CCR_PINC;    //DON'T INCREMENT PERIPHERAL
    DMA1_Channel4->CCR &= ~DMA_CCR_CIRC;    //DISABLE CIRCULAR MODE
    DMA1_Channel4->CCR &= ~DMA_CCR_MEM2MEM; //MEMORY TO PERIPHERAL
    DMA1_Channel4->CCR &= ~DMA_CCR_PL;      //LOW PRIORITY

    DMA1_Channel4->CCR |= DMA_CCR_TEIE | DMA_CCR_TCIE;  //ENABLE INTERRUPTS

    USART2->CR3 |= USART_CR3_DMAT;          //ENABLE DMA FOR TRANSMIT

    NVIC->ISER[0] = 1<<DMA1_Channel4_5_IRQn;  //ENABLE INTERRUPTS FOR CHANNELS 4 & 5
}

void DMA1_Channel4_5_IRQHandler(void)
{

    //CHANNEL 4
    if (DMA1->ISR & DMA_ISR_GIF4)
        DMA1->IFCR |= DMA_IFCR_CGIF4;       //CLEAR GLOBAL INTERRUPT FLAG

    if (DMA1->ISR & DMA_ISR_TCIF4)
        DMA1->IFCR |= DMA_IFCR_CTCIF4;      //CLEAR TRANSFER COMPLETE INTERRUPT FLAG

    if (DMA1->ISR & DMA_ISR_HTIF4)
        DMA1->IFCR |= DMA_IFCR_CHTIF4;       //CLEAR HALF TRANSFER COMPLETE INTERRUPT FLAG

    if (DMA1->ISR & DMA_ISR_TEIF4)
        DMA1->IFCR |= DMA_IFCR_CTEIF4;       //CLEAR TRANSFER ERROR INTERRUPT FLAG

}

void USART2_IRQHandler()
{
    if (USART2->ISR & USART_ISR_RXNE)      //RX REGISTER NOT EMPTY
    {
        char chartoreceive = (uint8_t)(USART2->RDR);
        usart2_rx_message[usart2_rx_index] = chartoreceive;
        usart2_rx_index++;

        USART2->RQR |= USART_RQR_RXFRQ;     //CLEAR INTERRUPT
    }

    if(USART2->ISR & USART_ISR_RTOF)
    {
        usart2_rx_index = 0;

    }

    if(USART2->ISR & USART_ISR_ORE)
        USART2->ICR |= USART_ICR_ORECF;     //CLEAR OVERRUN INTERRUPT

}

void send_usart2(char *usart2_tx_dma_buffer)
{
    uint32_t timeout = -1;

    while(DMA1->ISR & DMA_ISR_TCIF4);           //WAIT FOR TRANSFER COMPLETE FLAG TO BE CLEARED
    while(!(USART2->ISR & USART_ISR_TC))        //WAIT FOR TRANSMISSION COMPLETE
    {
        timeout++;
        if(timeout > 10000)
            return;
    }
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel4->CMAR = (uint32_t) usart2_tx_dma_buffer;
    DMA1_Channel4->CPAR = (uint32_t) &(USART2->TDR);
    DMA1_Channel4->CNDTR = strlen(usart2_tx_dma_buffer);
    DMA1_Channel4->CCR |= DMA_CCR_EN;
    timeout = 0;
    while(!(USART2->ISR & USART_ISR_TC))        //WAIT FOR TRANSMISSION COMPLETE
    {
        timeout++;
        if(timeout > 10000)
            return;
    }
    return;
}
