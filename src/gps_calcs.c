/**
  ******************************************************************************
  * @file    gps_calcs.c
  * @author  Ac6
  * @version V1.0
  * @date    31-March-2019
  * @brief   GPS_Calcs functions.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include <math.h>
#include "string.h"

#define slope_line -0.011204595048437    //slope of the finish line
#define b_line 43.997761000695430        //y intercept of finish line
#define slope_perp 89.249097863600970    //slope of perpendicular of finish line
#define solver_x 89.260302458649406      //used for finding intersection point

#define start_u 43.036180555555550
#define start_l 85.834791666666660
#define start_r 85.834550000000000
#define start_b 43.035861111111110

double b_perp = 0.0; //y intercept, where the two lines cross

/* Convert the incoming GPS coordinates to an "x and y" */
void convert_GPS(char* latitude, char* longitude, double* gps_x, double* gps_y)
{
    int gps_degree = 0;
    int gps_minute = 0;
    int i = 0;

    if(latitude[4] == '\0')     //check if string is good first
        return;

    if(longitude[4] == '\0')
        return;

    while(i < 3)
    {
        gps_degree = gps_degree*10 + (longitude[i] - '0');
        i++;
    }

    while(i < 5)
    {
        gps_minute = gps_minute*10 + (longitude[i] - '0');
        i++;
    }

    i++;

    while(i < 10)
    {
        gps_minute = gps_minute*10 + (longitude[i] - '0');
        i++;
    }

    *gps_x = gps_degree + (gps_minute / 600000.0);

    i = 0;

    gps_degree = 0.0;
    gps_minute = 0.0;

    while(i < 2)
    {
        gps_degree = gps_degree*10 + (latitude[i] - '0');
        i++;
    }

    while(i < 4)
    {
        gps_minute = gps_minute*10 + (latitude[i] - '0');
        i++;
    }

    i++;

    while(i < 9)
    {
        gps_minute = gps_minute*10 + (latitude[i] - '0');
        i++;
    }

    *gps_y = gps_degree + (gps_minute / 600000.0);

    return;
}

/* Check if the coordinates are close to the finish line */
int check_in_range(double gps_x, double gps_y)
{

    if(((gps_x - start_l) < 0) && ((gps_x - start_r) > 0))
        if(((gps_y - start_b) > 0) && ((gps_y - start_u) < 0))
            return 1;
    return 0;
}

/* Compute the distance to the finish line */
double distance_to_finish_line(double gps_x, double gps_y)
{
    double x_1 = 0.0;    //point on finish line
    double y_1 = 0.0;
    double distance = 0.0;   //distance to finish line

    b_perp = gps_y - (gps_x * slope_perp);

    x_1 = (b_line - b_perp) / solver_x;
    y_1 = (slope_line * x_1) + b_line;

    //distance = pow(pow(gps_x - x_1, 2) + pow(gps_y - y_1, 2),0.5);
    distance = pow(gps_x - x_1, 2) + pow(gps_y - y_1, 2);
    return distance;
}
