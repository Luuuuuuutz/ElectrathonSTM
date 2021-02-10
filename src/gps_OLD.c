
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <GPS.h>


char nmea1[MAXLINELENGTH] = {'\0'};
char nmea2[MAXLINELENGTH] = {'\0'};
int nmea_done; //most recent nmea

void init_dma5(char *nmea)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CMAR = (uint32_t) nmea;
    DMA1_Channel5->CPAR = (uint32_t) (&(USART2->RDR));
    DMA1_Channel5->CNDTR = MAXLINELENGTH;
    DMA1_Channel5->CCR &= ~DMA_CCR_DIR;
    DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR &= ~DMA_CCR_PINC;
    //DMA1_Channel5->CCR |= DMA_CCR_CIRC;

    DMA1_Channel5->CCR |= DMA_CCR_EN;
}
/*
void dma_usart2_init(void)
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

}


void send_USART2(const char *usart2_tx_dma_buffer)
{
    uint32_t timeout = -1;

    while(DMA1->ISR & DMA_ISR_TCIF2);           //WAIT FOR TRANSFER COMPLETE FLAG TO BE CLEARED
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
*/
void receive_sentence(char *nmea)
{
    init_dma5(nmea);
}
/*
void usart2_init(int baud)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(3<<(2*2));    //TX
    GPIOA->MODER |= 2<<(2*2);

    GPIOA->MODER &= ~(3<<(3*2));    //RX
    GPIOA->MODER |= 2<<(3*2);

    GPIOA->AFR[0] &= ~(0xf << (4*2));   //TX
    GPIOA->AFR[0] |= 0x1 << (4*2);

    GPIOA->AFR[0] &= ~(0xf << (4*3));   //RX
    GPIOA->AFR[0] |= 0x1 << (4*3);

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->CR1 &= ~USART_CR1_UE;
    USART2->CR1 &= ~USART_CR1_M;
    USART2->CR1 |= USART_CR1_CMIE;
    USART2->CR1 &= ~USART_CR1_PCE;
    USART2->CR1 &= ~USART_CR1_OVER8;
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->CR2 &= ~USART_CR2_ADD;
    USART2->CR2 |= 0x0a << 24;		//character match \n
    USART2->CR3 |= USART_CR3_DMAR;
    USART2->CR3 |= USART_CR3_DMAT;
    USART2->BRR = (48000000+1)/(baud-1);
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART2->CR1 |= USART_CR1_UE;
    NVIC->ISER[0] = 1 << USART2_IRQn;
    while((USART2->ISR & USART_ISR_TEACK) == 0);
    while((USART2->ISR & USART_ISR_REACK) == 0);

}

void USART2_IRQHandler(void) {
    USART2->CR3 &= ~USART_CR3_DMAR;
    USART2->ICR &= ~USART_ICR_CMCF;
    if (nmea_done == 2){
        nmea_done = 1;
        usart2_init(9600);
        receive_sentence(nmea2);
    }else{
        nmea_done = 2;
        usart2_init(9600);
        receive_sentence(nmea1);
    }

}
*/
