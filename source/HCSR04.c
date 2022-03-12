/*
 * HCSR04.c
 *
 *  Created on: 6 mar 2022
 *      Author: Enrique
 */
#include "HCSR04.h"

float usRanging()
{
    int time = 0;
    float cm = 0;
    do
    {
        gioSetBit(US_PORT, TRIGGER, 1);
        vTaskDelay(10/portTICK_RATE_US);
        gioSetBit(US_PORT, TRIGGER, 0);
    }
    while(gioGetBit(US_PORT, ECHO)==0);
    while(gioGetBit(US_PORT, ECHO)==1)
    {
        time++;
        vTaskDelay(1/portTICK_RATE_US);
    }
    cm = 0.1*((float)time);   // 5.9*s   El 5.9 inicial es un factor de correción
    return cm;
}
