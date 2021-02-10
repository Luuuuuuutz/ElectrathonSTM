/**
  ******************************************************************************
  * @file    adc.c
  * @author  Ac6
  * @version V1.0
  * @date    25-March-2019
  * @brief   ADC Configuration Functions.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"



void adc_init(void);
void dma_adc_init(void);
int adc_read(unsigned int);

extern uint16_t adc_value[3];      //adc readings array
extern uint8_t adc_counter;    //counter for 10 adc readings



void adc_init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB->MODER |= 3<<(2*0);       //set to analog (PB0, PC4, PC5)
    GPIOC->MODER |= 3<<(2*4);
    GPIOC->MODER |= 3<<(2*5);

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     //ENABLE CLOCK TO ADC1
    RCC->CR2 |= RCC_CR2_HSI14ON;            //SET CLOCK TO HIGH SPEED INTERNAL CLOCK

    ADC1->CFGR1 &= ~ADC_CFGR1_CONT;         //CONTINUOUS CONVERSION MODE
    ADC1->CFGR1 &= ~ADC_CFGR1_DISCEN;       //DISCONTINUOUS MODE
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;        //RIGHT ALIGNED
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;          //12 BIT RESOLUTION

    //ADC1->IER |= ADC_IER_EOCIE;             //ENABLE END OF CONVERSION INTERRUPT

    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));  //WAIT FOR THE CLOCK TO BE READY
    ADC1->CR |= ADC_CR_ADEN;                //ENABLE ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    //WAIT FOR ADC TO BE READY
    while((ADC1->CR & ADC_CR_ADSTART));     //WAIT FOR ADCSTART TO BE 0

    NVIC->ISER[0] = 1<<ADC1_COMP_IRQn;      //ENABLE INTERRUPTS FOR ADC1
}

void dma_adc_init()
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;       //ENABLE DMA CLOCK

    //DMA ADC
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;      //DISABLE TO MAKE CHANGES
    DMA1_Channel1->CMAR = (uint32_t) adc_value;
    DMA1_Channel1->CPAR = (uint32_t) &(ADC1->DR);
    DMA1_Channel1->CNDTR = sizeof adc_value / sizeof adc_value[0];
    DMA1_Channel1->CCR &= ~DMA_CCR_DIR;     //READ FROM PERIPHERAL
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;  //RESET DATA SIZE TO 8 BITS
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;  //RESET DATA SIZE TO 8 BITS
    DMA1_Channel1->CCR |= DMA_CCR_MINC;     //INCREMENT MEMORY
    DMA1_Channel1->CCR &= ~DMA_CCR_PINC;    //DON'T INCREMENT PERIPHERAL
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;     //ENABLE CIRCULAR MODE
    DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM; //MEMORY TO PERIPHERAL
    DMA1_Channel1->CCR &= ~DMA_CCR_PL;      //LOW PRIORITY

    DMA1_Channel1->CCR |= DMA_CCR_TEIE | DMA_CCR_TCIE;  //ENABLE INTERRUPTS

    ADC1->CFGR1 |= ADC_CFGR1_DMACFG;        //CIRCULAR MODE FOR ADC DMA
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;         //ENABLE DMA FOR ADC

    DMA1_Channel1->CCR |= DMA_CCR_EN;

    NVIC->ISER[0] = 1<<DMA1_Channel1_IRQn;  //ENABLE INTERRUPTS FOR CHANNEL 1
}

int adc_read(unsigned int channel)
{
    ADC1->CHSELR = 0;                       //DESELECT CHANNEL
    ADC1->CHSELR |= 1 << channel;           //SELECT CHANNEL
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    //WAIT FOR ADC READY
    ADC1->CR |= ADC_CR_ADSTART;             //START THE ADC
    while(!(ADC1->ISR & ADC_ISR_EOC));      //WAIT FOR END OF CONVERSION
    return ADC1->DR;
}

