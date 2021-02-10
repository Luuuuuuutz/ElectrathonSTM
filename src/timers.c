/**
  ******************************************************************************
  * @file    timers.c
  * @author  Ac6
  * @version V1.0
  * @date    30-March-2019
  * @brief   Timer Configuration Functions.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"



void race_timer_init(void);
void TIM2_IRQHandler(void);
void adc_timer_init(void);
void TIM3_IRQHandler(void);
void display_timer_init(void);
extern int adc_read(unsigned int);

extern int current_time;
extern int current_hunds;
extern int current_lap_hunds;
extern int voltage_update_counter;    //counter to update voltage
extern int update_display_flag;
extern int plot_voltage_flag;
extern int update_lap_time_flag;
extern int log_data_flag;
extern uint16_t adc_battery_voltage[100];
extern uint16_t adc_motor_voltage[100];
extern uint16_t adc_amperage[100];
uint8_t adc_index = 0;



void race_timer_init()
{
    current_time = 0;
    //TIM2 FOR RACE TIMER
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     //ENABLE TIM2

    TIM2->CR1 &= ~TIM_CR1_CEN;  //DISABLE TIMER2
    TIM2->CNT = 0;              //reset to 0
    TIM2->PSC = 480 - 1;
    TIM2->ARR = 1000 - 1;       //100 INTERRUPTS PER SECOND
    TIM2->DIER = TIM_DIER_UIE;  //ENABLE INTERRUPT
    TIM2->CR1 |= TIM_CR1_CEN;  //ENABLE TIMER2

    NVIC->ISER[0] = 1<<TIM2_IRQn;
}

void TIM2_IRQHandler()
{
    current_lap_hunds += 1;
    update_lap_time_flag = 1;
    current_hunds += 1;    //INCREMENT HUNDS SECONDS
    if(current_hunds >= 100)
    {
        current_hunds = 0;
        current_time += 1;                	//INCREMENT CURRENT TIME IN SECONDS
        voltage_update_counter +=  1;     	//INCREMENT ADC COUNTER
        log_data_flag = 1;					//data log
    }
    if (voltage_update_counter >= 10)
    {
        voltage_update_counter = 0;
        plot_voltage_flag = 1;
    }
    TIM2->SR = 0;                       //CLEAR INTERRUPT
}

void adc_timer_init()
{
    //TIM3 FOR TRIGGERING ADC
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     //ENABLE TIM3

    TIM3->PSC = 480 - 1;
    TIM3->ARR = 1000 - 1;       //100 INTERRUPTS PER SECOND

    TIM3->DIER = TIM_DIER_UIE;  //ENABLE INTERRUPT

    TIM3->CR1 |= TIM_CR1_CEN;   //ENABLE TIMER3

    NVIC->ISER[0] = 1<<TIM3_IRQn;
}

void TIM3_IRQHandler()
{
    adc_battery_voltage[adc_index] = adc_read(14);      //READ ADC AND PLACE IN PROPER ARRAY
    adc_motor_voltage[adc_index] = adc_read(15);
    adc_amperage[adc_index] = adc_read(8);

    adc_index++;
    if(adc_index > (sizeof adc_battery_voltage / sizeof adc_battery_voltage[0]))
            adc_index = 0;

    TIM3->SR = 0;       //CLEAR INTERRUPT
}

void display_timer_init()
{
    current_time = 0;
    //TIM2 FOR RACE TIMER
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;     //ENABLE TIM14

    TIM14->PSC = 4800 - 1;
    TIM14->ARR = 1000 - 1;       //10 INTERRUPTs PER SECOND

    TIM14->DIER = TIM_DIER_UIE;  //ENABLE INTERRUPT

    TIM14->CR1 |= TIM_CR1_CEN;  //ENABLE TIMER2

    NVIC->ISER[0] = 1<<TIM14_IRQn;
}

void TIM14_IRQHandler()
{
    update_display_flag = 1;

    TIM14->SR = 0;          //CLEAR INTERRUPT
}
