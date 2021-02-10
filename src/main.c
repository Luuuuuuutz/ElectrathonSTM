/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <main.h>


int main(void)
{
    //send_USART1("bauds=115200\xff\xff\xff");      //used to change baud rate for touch screen
    clear_all_interrupts();

    micro_wait(1000);   //wait a bit for GPS and touch screen to turn on

    usart1_init(9600);       //initialize usart at 115200 baud
    dma_usart1_init();

    display_timer_init();

    adc_init();
    adc_timer_init();

    sd_init();  //initialize the sd card

    usart2_init(9600);		//initialize GPS usart for 9600baud
    dma_usart2_init();

    //receive_sentence(nmea1);    //start receiving GPS strings

    send_usart2(PMTK_SET_NMEA_OUTPUT_RMCONLY);  //RMC only
    micro_wait(1000);
    send_usart2(PMTK_SET_NMEA_UPDATE_10HZ);     //update rate to 10Hz

    while(1)
    {
        if(usart1_rx_complete_flag)        //check for a complete touch screen message
        {
            usart1_rx_complete_flag = 0;       //clear the flag for another message

            if(usart1_rx_message[2] == 2)
            {
                load_drain();
                memset(usart1_rx_message, 0, sizeof usart1_rx_message);   //clear string for next message
            }

            if(usart1_rx_message[2] == 1)
            {
                race_mode = 1 ^ race_mode; //toggle the current mode

                update_start_button(race_mode);

                current_lap_hunds = 0;
                race_timer_init();

                if(race_mode)
                {
                    current_hunds = 0;
                    current_time = 0;
                    lap_counter = 0;
                    update_laps(lap_counter);
                }
                memset(usart1_rx_message, 0, sizeof usart1_rx_message);
            }

            if(usart1_rx_message[2] == 16) //radio button 1
            {
                if(race_mode == 0)  //only allowed to change if not in race mode
                {
                    update_radio_button(1); //race 1 button was pressed
                    race_number = 1;
                }
            }

            if(usart1_rx_message[2] == 17)  //radio button 2
            {
                if(race_mode == 0)
                {
                    update_radio_button(2);
                    race_number = 2;
                }
            }
        }


        if((!(current_time == prev_time)) && race_mode)     //if current second =! previous second and is in race mode then update the time on the touch screen
        {
            prev_time = current_time;
            update_time(current_time);
        }

        if(plot_voltage_flag && race_mode)
        {
            plot_voltage_flag = 0;
            //battery_voltage = voltage_average(adc_battery_voltage, (sizeof adc_battery_voltage / sizeof adc_battery_voltage[0])) / 4096.0 * 3.0 * 10;
            plot_voltage(battery_voltage);
        }

        /* UPDATE CURRENT LAPTIME */
        if(update_lap_time_flag && race_mode)
        {
            update_lap_time_flag = 0;
            update_lap_time(1, current_lap_hunds);     //1 for current lap time, 2 for prev lap time
        }

        /*UPDATE PREVIOUS LAPTIME*/
        if(lap_complete_flag)
        {
            lap_counter++;
            update_laps(lap_counter);
            lap_complete_flag = 0;
            prev_lap_hunds = current_lap_hunds;
            update_lap_time(2, prev_lap_hunds);     //1 for current lap time, 2 for prev lap time
            current_lap_hunds = 0;                  //reset current lap time to 0 hund seconds
        }

        /*UPDATE VOLTAGES ON DISPLAY*/
        if(update_display_flag)
        {
            update_display_flag = 0;

            battery_voltage = voltage_average(adc_battery_voltage, (sizeof adc_battery_voltage / sizeof adc_battery_voltage[0])) / 4096.0 * 3.0 * 10;
            update_battery_voltage(battery_voltage);

            motor_voltage = voltage_average(adc_motor_voltage, (sizeof adc_motor_voltage / sizeof adc_motor_voltage[0])) / 4096.0 * 3.0 * 10;
            update_motor_voltage(motor_voltage);

            amperage = voltage_average(adc_amperage, (sizeof adc_amperage / sizeof adc_amperage[0])) / 4096.0 * 3.0 * 4.7;
            update_amperage(amperage);

            if(nmea_done == 1)
                parse_gps(nmea1, &latitude, &longitude, &speed, MAXLINELENGTH);
            else
                parse_gps(nmea2, &latitude, &longitude, &speed, MAXLINELENGTH);

            speedmph = strtod(speed, NULL);
            speedmph = speedmph * 1.150779;
            update_velocity(speedmph);

        }

        /*LOG DATA TO SD CARD*/
        if(log_data_flag)
        {
            FRESULT res;    //result of the FAT operation
            UINT btw;
            UINT bw;

            log_data_flag = 0;

            battery_voltage_whole = (int) battery_voltage;
            battery_voltage_dec = (battery_voltage - battery_voltage_whole) * 100;

            motor_voltage_whole = (int) motor_voltage;
            motor_voltage_dec = (motor_voltage - motor_voltage_whole) * 100;

            amperage_whole = (int) amperage;
            amperage_dec = (amperage - amperage_whole) * 100;

            speed_whole = (int) speedmph;
            speed_dec = (speedmph - speed_whole) * 100;

            sprintf(data_packet, "%2d.%02d,%2d.%02d,%2d.%02d,%2d.%02d,%sN,%sW,%d\r\n",battery_voltage_whole, battery_voltage_dec,
            		motor_voltage_whole, motor_voltage_dec, amperage_whole, amperage_dec, speed_whole, speed_dec, latitude,
					longitude, current_time);

            btw = strlen(data_packet);

            f_mount(&FatFs, "", 0);     /* Give a work area to the default drive */

            if(race_number == 1)    //check which race is picked
                strcpy(file_name,"race1.txt");
            if(race_number == 2)
                strcpy(file_name,"race2.txt");

            res = f_open(&Fil, file_name, FA_WRITE | FA_OPEN_APPEND);
            if(res == FR_OK)
            {
                res = f_write(&Fil, data_packet, btw, &bw); //write the data packet
            }
            f_close(&Fil);
        }

        //lap determination logic
        if((nmea_done != nmea_last) && race_mode && use_lap_logic)  //process the newest GPS string only if it is turned on
        {
            convert_GPS(latitude, longitude, &gps_x, &gps_y);
            int in_range = check_in_range(gps_x, gps_y);

            if((in_range == 1) && (!recent_lap))
            {
                distance = distance_to_finish_line(gps_x, gps_y);
                difference = distance - last_distance;
                last_distance = distance;

                if((difference - last_difference) > 0)
                {
                    direction = 0;
                    increasing_num++;
                }

                if((difference - last_difference) < 0)
                {
                    direction = 1;
                    decreasing_num++;
                }

                if(distance == 0.0 || ((direction != last_direction) && (increasing_num > 2) && (decreasing_num > 2))) //if the current direction is not the same as the previous direction then we must have crossed the finish line
                {
                    increasing_num = 0; //reset counters
                    decreasing_num = 0;
                    lap_complete_flag = 1;  //set lap complete flag
                    recent_lap = 1;
                }

                last_difference = difference;
                last_direction = direction;
            }

            if(in_range == 0)   //break "latch" once we move beyond the finish zone
            {
                recent_lap = 0;
            }
        }

    }

    asm("wfi");
    for(;;);

}

/* Clears all interrupts at start up to attempt to get it out of a potential bad state */
void clear_all_interrupts()
{
    USART1->ISR = 0;
    USART2->ISR = 0;
    DMA1->ISR = 0;
    ADC1->ISR = 0;
    SPI2->SR = 0;
    TIM2->SR = 0;
    TIM3->SR = 0;
}

/* Load the battery drain file from the SD card and plot it on the graph */
void load_drain()
{
    char drain_data[7] = {'\0'};
    char cmd[16];
    int i = 0;

    FRESULT res;    //result of the FAT operation

    UINT btr = sizeof drain_data / sizeof drain_data[0];    //number of bytes to read
    UINT br;    //pointer to keep track of where in the file you are

    int result = sd_init();     // try initializing the card with my own code, which seems more successful

    if(result || 0)     //check if initialization was successful
        return;

    f_mount(&FatFs, "", 0);     /* Give a work area to the default drive */

    res = f_open(&Fil, "drain.txt", FA_READ | FA_OPEN_EXISTING); /* Open a file*/
    if(res) // if not 0 (success) then return
        return;

    while(res == FR_OK && !f_eof(&Fil))     //continue to read the file until end of file
    {
        micro_wait(5);
        res = f_read(&Fil, &drain_data, btr, &br);  //read contents from file
        micro_wait(5);
        drain_data[5] = '\0';       //terminate the string early to remove new line and return
        double voltage = strtod(drain_data, NULL);

        int xcoord = i + 118;
        int ycoord = 57 + ((26.0-voltage) * 25.0);

        sprintf(cmd, "pic %d,%d,0\xff\xff\xff", xcoord, ycoord);
        send_usart1(cmd);
        i++;
    }
    f_close(&Fil);      /* Close the file */

    return;
}

/* Plot a live voltage point on the graph */
void plot_voltage(float voltage)
{
    char cmd[16];

    int xcoord = (current_time/10) + 118;
    int ycoord = 57 + ((26.0-voltage) * 25.0);

    bv_graph_log[(int) (current_time/10)] = ycoord;

    sprintf(cmd, "pic %d,%d,1\xff\xff\xff", xcoord, ycoord);
    send_usart1(cmd);

    return;
}

/*Replot the entire voltage log*/
void plot_log(uint16_t * voltage_array, int size)
{
    char cmd[16];
    int i = 0;

    load_drain();   //First load the battery curve

    for(i = 0; i<100; i++)
    {
        if (voltage_array[i] > 0)
        {
            int xcoord = 118 + i;
            int ycoord = 57 + ((26.0-voltage_array[i]) * 25.0);

            sprintf(cmd, "pic %d,%d,1\xff\xff\xff", xcoord, ycoord);
            send_usart1(cmd);
        }
    }

    return;
}

/* Update the current battery voltage */
void update_battery_voltage(float voltage)
{
    char cmd[26];
    int voltage_whole = (int) voltage;
    int voltage_dec = (voltage - voltage_whole) * 100;

    sprintf(cmd, "batterytxt.txt=\"%2d.%02d\"\xff\xff\xff", voltage_whole, voltage_dec);
    send_usart1(cmd);

    return;
}

/* Update the current motor voltage */
void update_motor_voltage(float voltage)
{
    char cmd[25];
    int voltage_whole = (int) voltage;
    int voltage_dec = (voltage - voltage_whole) * 100;

    sprintf(cmd, "motortxt.txt=\"%2d.%02d\"\xff\xff\xff", voltage_whole, voltage_dec);
    send_usart1(cmd);

    return;
}

/* Update the current amperage draw */
void update_amperage(float voltage)
{
    char cmd[27];
    int voltage_whole = (int) voltage;
    int voltage_dec = (voltage - voltage_whole) * 100;

    sprintf(cmd, "amperagetxt.txt=\"%2d.%02d\"\xff\xff\xff", voltage_whole, voltage_dec);
    send_usart1(cmd);

    return;
}

/* Update the current velocity */
void update_velocity(float speed)
{
    char cmd[27];
    int speed_whole = (int) speed;
    int speed_dec = (speed - speed_whole) * 100;

    sprintf(cmd, "velocitytxt.txt=\"%2d.%02d\"\xff\xff\xff", speed_whole, speed_dec);
    send_usart1(cmd);

    return;
}

/*Update the current race time */
void update_time(int seconds)
{
    char cmd[23];
    uint16_t minutes;
    minutes = seconds / 60;
    seconds = seconds - (minutes * 60);

    sprintf(cmd, "timetxt.txt=\"%02d:%02d\"\xff\xff\xff", minutes, seconds);
    send_usart1(cmd);

    return;
}

/* Update the current lap time */
void update_lap_time(int txt, int hunds_seconds)
{
    char cmd[41];
    int minutes;
    int seconds;
    minutes = (hunds_seconds / 6000);
    seconds = (hunds_seconds - (minutes * 6000)) / 100;
    hunds_seconds = hunds_seconds - (minutes * 6000) - (seconds * 100);

    if(txt == 1)
        sprintf(cmd, "currentlaptxt.txt=\"Current: %d:%02d.%02d\"\xff\xff\xff", minutes, seconds, hunds_seconds);
    else if (txt == 2)
        sprintf(cmd, "prevlaptxt.txt=\"Last: %d:%02d.%02d\"\xff\xff\xff", minutes, seconds, hunds_seconds);

    send_usart1(cmd);
    return;
}

/* Compute the average voltage/amperage in the array */
int voltage_average(uint16_t * voltage_array, int size)
{
    int average = 0;
    for(int i = 0; i < size; i++)
    {
        average = average + voltage_array[i];
    }
    average = average / size;

    return average;
}

/* Update lap counter on touch screen */
void update_laps(uint8_t laps)
{
    char cmd[24];
    sprintf(cmd, "lapstxt.txt=\"Laps:%d\"\xff\xff\xff", laps);
    send_usart1(cmd);
}

/* Update start button text */
void update_start_button(int race_mode)
{
    char cmd[29];

    if(race_mode)
    {
        sprintf(cmd, "startbtn.txt=\"Stop Race\"\xff\xff\xff");
        send_usart1(cmd);
    }
    else
    {
        sprintf(cmd, "startbtn.txt=\"Start Race\"\xff\xff\xff");
        send_usart1(cmd);
    }
}

/* Update which radio button is pressed for the race number */
void update_radio_button(int race_num)
{
    char cmd[21];

    if(race_num == 1)
    {
        sprintf(cmd, "radiobtn1.val=1\xff\xff\xff");
        send_usart1(cmd);
        sprintf(cmd, "radiobtn2.val=0\xff\xff\xff");
        send_usart1(cmd);
    }
    else
    {
        sprintf(cmd, "radiobtn1.val=0\xff\xff\xff");
        send_usart1(cmd);
        sprintf(cmd, "radiobtn2.val=1\xff\xff\xff");
        send_usart1(cmd);
    }

    return;
}

/* Update the radio button to show if the GPS has a fix or not */
void update_gps_fix(int fix)
{
    char cmd[21];

    if(fix == 1)
    {
        sprintf(cmd, "radiobtn3.val=1\xff\xff\xff");
        send_usart1(cmd);
    }
    else if(fix == 0)
    {
        sprintf(cmd, "radiobtn3.val=0\xff\xff\xff");
        send_usart1(cmd);
    }
}

/* Parse the newest GPS string */
void parse_gps(char* string, char* lat, char* lon, char* speed, int len)
{
    //int start_found = 1;
	int i = 0;
	int comma_pos = 0;
	int num_comma = 0;
	GPS_fix = 0;

	if(string[0] != '$')
	    goto end;

	if(string[18] == 'A')
	{
	    GPS_fix = 1;

	    while(i <= len)
	    {
	        if(string[i] == ',')
	        {
	            num_comma++;
	            comma_pos = i;
	        }
	        else if(num_comma == 3)
	        {
	            lat[i - comma_pos - 1] = string[i];
	        }
	        else if(num_comma == 5)
	        {
	            lon[i - comma_pos - 1] = string[i];
	        }
	        else if(num_comma == 7)
	        {
	            speed[i - comma_pos - 1] = string[i];
	        }

	        if(num_comma == 8)
	            goto end;

	        i++;
	    }
	}
	else    //send 0s if not fixed
	{
	    lat[0] = '\0';
	    lon[0] = '\0';
	    speed[0] = '\0';
	}

	end:
    if(GPS_fix != last_fix)
    {
        update_gps_fix(GPS_fix);
    }
	last_fix = GPS_fix;
	return;
}

