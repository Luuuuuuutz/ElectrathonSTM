#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include <ff.h>     /* Declarations of FatFs API */
#include <GPS.h>

FATFS FatFs;        /* FatFs work area needed for each volume */
FIL Fil;            /* File object needed for each open file */

#define bool int
#define true 1
#define false 0

#define use_lap_logic 1

void load_drain(void);
void plot_voltage(float);
void plot_log(uint16_t*, int);
void update_battery_voltage(float);
void update_motor_voltage(float);
void update_amperage(float);
void update_velocity(float);
void update_time(int);
void update_lap_time(int, int);
void update_laps(uint8_t);
void update_start_button(int);
void update_radio_button(int);
void update_gps_fix(int);
void clear_all_interrupts(void);
int voltage_average(uint16_t*, int);
void parse_gps(char*, char*, char*, char*, int);

extern void usart1_init(int);
extern void dma_usart1_init(void);
extern void send_usart1(char *);
extern void usart2_init(int);
extern void dma_usart2_init(void);
extern void send_usart2(char *);

extern void race_timer_init(void);
extern void adc_timer_init(void);
extern void display_timer_init(void);

extern void adc_init(void);
extern void dma_adc_init(void);

extern uint8_t sd_init(void);
extern uint8_t sd_read(unsigned long, unsigned short, unsigned char*, unsigned short);

extern void usart2_init(int);
extern void dma_usart2_init(void);
extern void send_USART2(const char *);
extern void receive_sentence(char *);

extern void convert_gps(char*, char*, double*, double*);
extern int check_in_range(double, double);
extern double distance_to_finish_line(double, double);

char buffer[8];

char usart1_rx_message[16] = {};   //buffer
int usart1_rx_complete_flag = 0;   //flag for a complete touch screen message
char usart2_rx_message[MAXLINELENGTH] = {};   //buffer
int usart2_rx_complete_flag = 0;   //flag for a complete touch screen message

int current_time = 0;
int prev_time = 0;
int current_hunds = 0;
int current_lap_hunds = 0;
int prev_lap_hunds = 0;
int race_number = 1;

bool race_mode = false;     //in race mode or not

uint16_t adc_value[3];      //adc readings array
uint8_t adc_counter = 0;    //counter for 10 adc readings

uint16_t adc_battery_voltage[100];
uint16_t adc_motor_voltage[100];
uint16_t adc_amperage[100];

uint16_t bv_graph_log[360] ={0};

float battery_voltage = 0;
float motor_voltage = 0;
float amperage = 0;
float speedmph = 0;

int battery_voltage_whole;
int battery_voltage_dec;
int motor_voltage_whole;
int motor_voltage_dec;
int amperage_whole;
int amperage_dec;
int speed_whole;
int speed_dec;

char data_packet[55] = {'\0'};  //packet of information BV, MV, AMP, VEL, LAT, LONG, RACE TIME
char file_name[10] = {'\0'};
char GPSstring[120] = {'\0'};
char latitude[8] = {'\0'};
char longitude[9] = {'\0'};
char speed[5] = {'\0'};

double gps_x = 0;
double gps_y = 0;
double distance = 0;
double last_distance = 0;
double difference = 0;
double last_difference = 0;

int direction = 0;      //0 for increasing, 1 for decreasing
int last_direction = 0;
int increasing_num = 0;
int decreasing_num = 0;
int recent_lap = 1;
int lap_counter = 0;

extern char nmea1[MAXLINELENGTH];
extern char nmea2[MAXLINELENGTH];
extern int nmea_done; //most recent nmea
int nmea_last = 0;
int GPS_fix = 0;    //0 for not fixed, 1 for a fix
int last_fix = 1;

int voltage_update_counter = 0;
int update_display_flag = 0;
int plot_voltage_flag = 0;
int update_lap_time_flag = 0;
int lap_complete_flag = 0;
int log_data_flag = 0;

uint8_t return_status;




