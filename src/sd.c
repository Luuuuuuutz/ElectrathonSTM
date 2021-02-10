/**
  ******************************************************************************
  * @file    sd.c
  * @author  Ac6
  * @version V1.0
  * @date    27-March-2019
  * @brief   SD Card Functions.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <sd.h>

char test_buffer[100] = {0};

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

uint8_t sd_init()
{
	inBlock_ = offset_ = partialBlockRead_ = 0; //initialize variables
	int timeout = 0;    //timeout
	sd_startup();       //bit bang 74 clock cycles to start the card
    spi_init();         //initialize SPI2 settings

    while((status_=sd_cmd(CMD0, 0x00000000)) != R1_IDLE_STATE)	//cmd0 to go into idle state
    {
    	timeout++;
    	if(timeout > 100)   //up to 100 retries before failing
    		goto fail;
    }

    micro_wait(5);      //wait a moment before continuing

    sd_cmd(CMD8, 0x000001aa);	//cmd8 to check SD card version

    micro_wait(5);

    timeout = 0;
    while((status_ = sd_acmd(ACMD41, 0x00000000)) != R1_READY_STATE)	//send acmd41 to initialize sd card
    {
    	timeout++;
    	if(timeout > 1000)
    		goto fail;
    }

    return PASSED;

    fail:

	return FAILED;
}

int sd_startup()
{
    const int CS = 1<<4;
    const int SCK = 1<<5;
    const int MOSI = 1<<7;

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(3<<(4*2));		//CS
    GPIOA->MODER |= 1<<(4*2);

    GPIOA->MODER &= ~(3<<(5*2));       //SCK
    GPIOA->MODER |= 1<<(5*2);

    GPIOA->MODER &= ~(3<<(7*2));       //MOSI
    GPIOA->MODER |= 1<<(7*2);

    GPIOA->BSRR = CS;   //set CS bit
    GPIOA->BSRR = MOSI; //set MOSI bit

    for(int i = 0; i < 150; i++)    //send 150 clock cycles
    {
    	GPIOA->BSRR = SCK;
    	nano_wait(SPI_DELAY);
    	GPIOA->BRR = SCK;
    	nano_wait(SPI_DELAY);
    }

    return 1;
}

uint8_t sd_cmd(unsigned char cmd, unsigned long arg)
{
    uint8_t statuscmd_;	//return status from the sd card

    char crc = 0xff;	//crc is 0xff unless it is command 0 or 8
    if(cmd == CMD0)
    	crc = 0x95;		//crc is 0x95 for command 0
    if(cmd == CMD8)
    	crc = 0x87;		//crc is 0x87 for command 8

    spi_write(0xff);		//send 0xff for the sd card to get "ready" for a command

    spi_write(cmd | 0x40);	//send command
    spi_write(arg>>24);		//send arguments
    spi_write(arg>>16);
    spi_write(arg>>8);
    spi_write(arg);
    spi_write(crc);			//send CRC

    micro_wait(5);			//wait a bit for the sd card to be ready
    for(int i = 0; (((statuscmd_ = spi_write(0xff)) & 0xf0) && (i != 0x08)); i++);	//read from the sd card until a proper response comes in

    micro_wait(5);
    return statuscmd_;	//final response from the sd card
}

uint8_t sd_acmd(unsigned char acmd, unsigned long arg)
{
	sd_cmd(CMD55,0);			//every acmd has command 55 sent first, then the acmd is sent
	return sd_cmd(acmd, arg);
}


uint8_t wait_not_busy(unsigned int timeout_limit)
{
    unsigned int timeout = 0;
    do
    {
        if(spi_write(0xff) == 0xff)
            return 1;
        timeout++;
    }while(timeout < timeout_limit);

    return 0;
}
