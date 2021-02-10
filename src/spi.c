/**
  ******************************************************************************
  * @file    spi.c
  * @author  Ac6
  * @version V1.0
  * @date    25-March-2019
  * @brief   SPI Configuration Functions.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"



void spi_init(void);
char spi_write(char);

char sd_response_data;
char data_return;



void spi_init(void) {
    //ENABLE GPIO
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(3<<(4*2));       //NSS
    GPIOA->MODER |= 2<<(4*2);

    GPIOA->MODER &= ~(3<<(5*2));       //SCK
    GPIOA->MODER |= 2<<(5*2);

    GPIOA->MODER &= ~(3<<(6*2));       //MISO
    GPIOA->MODER |= 2<<(6*2);

    GPIOA->MODER &= ~(3<<(7*2));       //MOSI
    GPIOA->MODER |= 2<<(7*2);


    GPIOA->AFR[0] &= ~(0xf << (4*4));       //alternate function for PA4
    GPIOA->AFR[0] |= 0x0 << (4*4);

    GPIOA->AFR[0] &= ~(0xf << (4*5));       //alternate function for PA5
    GPIOA->AFR[0] |= 0x0 << (4*5);

    GPIOA->AFR[0] &= ~(0xf << (4*6));       //alternate function for PA6
    GPIOA->AFR[0] |= 0x0 << (4*6);

    GPIOA->AFR[0] &= ~(0xf << (4*7));       //alternate function for PA7
    GPIOA->AFR[0] |= 0x0 << (4*7);

    //ENABLE SPI
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_BIDIMODE;         //BIDIRECTIONAL MODE
    SPI1->CR1 &= ~SPI_CR1_BIDIOE;            //BIDIRECTIONAL MODE ENABLE
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;                //BAUD RATE (f/128)
    SPI1->CR1 |= SPI_CR1_MSTR;              //MASTER CONFIGURATION
    SPI1->CR1 &= ~SPI_CR1_CPOL;             //CK 0 WHEN IDLE
    SPI1->CR1 &= ~SPI_CR1_CPHA;             //FIRST CLOCK TRANSITION
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;         //MSB FIRST

    SPI1->CR2 &= ~SPI_CR2_DS;               //RESET TO 8 BIT WORD SIZE
    SPI1->CR2 |= SPI_CR2_NSSP;              //AUTO NSSP
    SPI1->CR2 |= SPI_CR2_RXNEIE;            //ENABLE SPI RX BUFFER NOT EMPTY INTERRUPT

    SPI1->CR2 |= SPI_CR2_FRXTH;

    SPI1->CR1 |= SPI_CR1_SPE;               //ENABLE SPI

}

void spi_update_speed()
{
    SPI1->CR1 &= ~SPI_CR1_SPE;  //disable SPI to change speed
    SPI1->CR1 &= ~SPI_CR1_BR;   //set to 6MHz speed
    SPI1->CR1 |= SPI_CR1_BR_1;
    SPI1->CR1 |= SPI_CR1_SPE;   //enable SPI
    return;
}

char spi_write(char b)
{
	int timeout = 0;

    while(!(SPI1->SR & SPI_SR_TXE))     //WAIT FOR TRANSMIT BUFFER TO BE EMPTY
    {
        timeout++;
        if(timeout > 1000)
            return 0xff;
    }

    *((uint8_t *)&(SPI1->DR)) = b;      //send data

    timeout = 0;
    while(((SPI1)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPI1)->SR & SPI_SR_BSY)) 	//wait for SPI to be no longer busy
    {
            timeout++;
            if(timeout > 1000)
                return 0xff;
    }

    return (uint8_t)SPI1->DR;       //receieve data
}

