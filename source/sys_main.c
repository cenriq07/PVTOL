/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "FreeRTOS.h"
#include "os_task.h"
#include "sys_core.h"
#include "gio.h"
#include "sci.h"
#include "spi.h"
#include "het.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
//Hola ivon :)

// Mientras tu escribes una cancion...
/*
 * int i=0;
 * for(i=0; i<0xFFFFFFF; i++)
 * {
 *      printf("Me caes bien");
 * }
 *
 * */
#include "MPU9250.h"
#include "HCSR04.h"
#include "PWM.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
/*
 *  .:| PVTOL |:.
 *
 *  Universidad Aeronáutica en Querétaro
 *  2021
 *
 *  Cortés Nava Luis Enrique
 *  5391
 *  IECSA07A
 *
 *  Técnicas de Identificación
 * */

/*------------------ DIRECTIVAS DE PREPROCESAMIENTO ---------------*/
//#define INIT        true    // <-------- Inicialización BLDC

/* SETROLL & SETALT solo debe estar habilitado UNO A LA VEZ */
#define SETROLL       TRUE
//#define SETALT        TRUE
#define SETPOWER        TRUE

/*--------------------- PID -----------------------*/
#ifdef SETROLL
float Kp_1 = 0.0;
float Ki_1 = 0.0;
float Kd_1 = 0.0;
#else
float Kp_1 = 0.33;
float Ki_1 = 0.001;
float Kd_1 = 0.2;
#endif

#ifdef SETALT
float Kp_2 = 0.0;
float Ki_2 = 0.0;
float Kd_2 = 0.0;
#else
float Kp_2 = 0.0;
float Ki_2 = 0.0;
float Kd_2 = 0.0;
#endif

#define setPoint_1    0.0     // <-------- ANGULO
#define setPoint_2    15.0     // <-------- ALTURA

float theta = 0.0;
float usDistance = 0.0;

float error_1 = 0.0;
float error_2 = 0.0;

float preverror_1 = 0.0;
float preverror_2 = 0.0;

void limitControl(int *u, int max, int min);
/*------------------- MOTORES --------------------*/
#define BLDC_1_PWM  pwm0    // HET 0
#define BLDC_2_PWM  pwm1    // HET 2
#define DC_MAX      1000
#define DC_MIN      530
int dcBASE = 545;
int offset1 = 0;
int offset2 = 0;

int DC_BLDC1 = 0, DC_BLDC2 = 0;

hetSIGNAL_t BLDC_1;
hetSIGNAL_t BLDC_2;

void bldcInit();

/*----------------- ULTRASONICO -------------------*/
#define TRIGGER     0
#define ECHO        1
#define US_PORT     gioPORTA
/*-------------------- SCI ----------------------*/
char sciBuffer[12];
static char BuffReceive[8];
char cadenaTemporal[6];
char buffer[6];
int sciKp_1 = 0, sciKi_1 = 0, sciKd_1 = 0;
int sciKp_2 = 0, sciKi_2 = 0, sciKd_2 = 0;

/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
void vSensors(void *pvParameters);
void vControl(void *pvParameters);

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    gioInit();
    sciInit();
    spiInit();
    hetInit();
    MPUInit();

    _enable_interrupt_();
    _enable_IRQ();

    /*   INTERRUPCION SCI    */
    sciEnableNotification(scilinREG, SCI_RX_INT);
    sciReceive(scilinREG,5,(unsigned char *)&BuffReceive);

    xTaskCreate(vSensors,"MPU",256,NULL,1,NULL);
    xTaskCreate(vControl,"PID",1024,NULL,1,NULL);

    vTaskStartScheduler();

    while(1);
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */

/*-------------------------- TAREAS ------------------------------*/
void vSensors(void *pvParameters)
{
    int32_t Gyro[3]={0,0,0};
    int32_t Accel[3]={0,0,0};
    float Ax = 0.0, Ay = 0.0, Az = 0.0;

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    float tmpTheta = 0.0;
    float tmpUSonic = 0.0;
    int i = 0;

    while(1)
    {

        mpuGetGyroAccel(spiREG1, &SPI1_data_configCh2,Gyro,Accel);

        Ax = Accel[X]*Wx + Vx;
        Ay = Accel[Y]*Wy + Vy;
        Az = Accel[Z]*Wz + Vz;

        if(i<10)
        {
            tmpTheta = tmpTheta + mpuGetAccAngle_Y(Ax, Ay, Az);
            tmpUSonic = tmpUSonic + usRanging();
            i++;
        }
        else
        {
            theta = 2 + tmpTheta/10.0;
            usDistance = tmpUSonic/10.0;
            tmpTheta = 0.0;
            tmpUSonic = 0.0;
            i = 0;
        }
        vTaskDelayUntil(&xLastWakeTime, 10/portTICK_RATE_MS);
    }
}

void vControl(void *pvParameters)
{
    float P_1 = 0.0, P_2 = 0.0;
    float I_1 = 0.0, I_2 = 0.0;
    float D_1 = 0.0, D_2 = 0.0;
    float PID_1 = 0.0, PID_2 = 0.0;

    /* Motor 1*/
    BLDC_1.period = 20000;
    BLDC_1.duty = 500;

    /* Motor 2 */
    BLDC_2.period = 20000;
    BLDC_2.duty = 500;

    pwmSetSignal10e3(hetRAM1, BLDC_1_PWM, BLDC_1);
    pwmSetSignal10e3(hetRAM1, BLDC_2_PWM, BLDC_2);

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

#ifdef INIT
    bldcInit();
#endif

    while(1)
    {
        error_1 = setPoint_1 - (int)theta;
        error_2 = setPoint_2 - (int)usDistance;
        /*~~~~~~~~~~~~~~~~~ Tuning ~~~~~~~~~~~~~~~~~~~~~*/
#ifdef SETROLL
        Kp_1 = ((float)sciKp_1)/1000.0;
        Ki_1 = ((float)sciKi_1)/1000.0;
        Kd_1 = ((float)sciKd_1)/1000.0;
#endif
#ifdef SETALT
        Kp_2 = ((float)sciKp_2)/1000.0;
        Ki_2 = ((float)sciKi_2)/1000.0;
        Kd_2 = ((float)sciKd_2)/1000.0;
#endif
        /*~~~~~~~~~~~~~~~~~ Proportional ~~~~~~~~~~~~~~~~~~~~~*/
        P_1 = Kp_1*error_1;
        P_2 = Kp_2*error_2;
        /*~~~~~~~~~~~~~~~~~ Integral ~~~~~~~~~~~~~~~~~~~~*/
//        if(setPoint_1-5<error_1<setPoint_1+5)
            I_1 = I_1 + (Ki_1*error_1/10.0);
//        else
//            I_1 = 0;

        I_2 = I_2 + (Ki_2*error_2);
        /*~~~~~~~~~~~~~~~~~ Derivative ~~~~~~~~~~~~~~~~~~~~~*/
        D_1 = Kd_1*1000.0*(error_1 - preverror_1)/50.0;
        D_2 = Kd_2*1000.0*(error_2 - preverror_2)/50.0;
        /*~~~~~~~~~~~~~~~~~~~~ PID ~~~~~~~~~~~~~~~~~~*/
        PID_1 = P_1 + I_1 + D_1;
        PID_2 = P_2 + I_2 + D_2;
        /*--------------------- Control -------------*/
        DC_BLDC1 = (dcBASE + (int)PID_2) - (int)PID_1 + offset1;
        DC_BLDC2 = (dcBASE + (int)PID_2) + (int)PID_1 + offset2;

        limitControl(&DC_BLDC1, DC_MAX, DC_MIN);
        limitControl(&DC_BLDC2, DC_MAX, DC_MIN);

        BLDC_1.duty = DC_BLDC1;
        BLDC_2.duty = DC_BLDC2;
        pwmSetSignal10e3(hetRAM1, BLDC_1_PWM, BLDC_1);
        pwmSetSignal10e3(hetRAM1, BLDC_2_PWM, BLDC_2);
        /*----------------------------------*/
        preverror_1 = error_1;
        preverror_2 = error_2;

#ifdef SETROLL
        sciSend(scilinREG, sprintf(sciBuffer,"/*%d,%d,%d,%d,%d,%d*/",offset1,DC_BLDC1,DC_BLDC2,(int)(Kp_1*1000),(int)(Ki_1*1000),(int)(Kd_1*1000)),(uint8*)sciBuffer);
#else
        sciSend(scilinREG, sprintf(sciBuffer,"/*%d,%d,%d,%d,%d*/",(int)(theta),(int)(usDistance),(int)(Kp_1*1000),(int)(Ki_1*1000),(int)(Kd_1*1000)),(uint8*)sciBuffer);
#endif
#ifdef SETALT
        sciSend(scilinREG, sprintf(sciBuffer,"/*%d,%d,%d,%d,%d*/",(int)(theta),(int)(usDistance),(int)(Kp_2*1000),(int)(Ki_2*1000),(int)(Kd_2*1000)),(uint8*)sciBuffer);
#endif

        vTaskDelayUntil(&xLastWakeTime, 40/portTICK_RATE_MS);
    }
}

/*-------------------------- FUNCIONES ------------------------------*/

void limitControl(int *u, int max, int min)
{
    if(*u > max)
        *u = max;
    if(*u < min)
        *u = min;
}

void bldcInit()
{
    BLDC_1.period = 20000;
    BLDC_2.period = 20000;
    int control = 0;

    BLDC_1.duty = 500;
    BLDC_2.duty = 500;
    pwmSetSignal10e3(hetRAM1, BLDC_1_PWM, BLDC_1);
    pwmSetSignal10e3(hetRAM1, BLDC_2_PWM, BLDC_2);

    while(control<3)
    {
        if(control == 0)
        {
            vTaskDelay(1000/portTICK_RATE_MS);
            control++;
        }
        if(control == 1)
        {
            BLDC_1.duty = 1000;
            BLDC_2.duty = 1000;
            pwmSetSignal10e3(hetRAM1, BLDC_1_PWM, BLDC_1);
            pwmSetSignal10e3(hetRAM1, BLDC_2_PWM, BLDC_2);
            vTaskDelay(3000/portTICK_RATE_MS);
            control++;
        }
        if(control == 2)
        {
            BLDC_1.duty = 500;
            BLDC_2.duty = 500;
            pwmSetSignal10e3(hetRAM1, BLDC_1_PWM, BLDC_1);
            pwmSetSignal10e3(hetRAM1, BLDC_2_PWM, BLDC_2);
            vTaskDelay(3000/portTICK_RATE_MS);
            control++;
        }
    }
}

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

/*-------------------------- INTERRUPCIONES ------------------------------*/

void sciNotification(sciBASE_t*sci,unsigned flags)
{
    sciReceive(scilinREG,5,(unsigned char *)&BuffReceive);

    if(BuffReceive[0] == 'P')
    {
       cadenaTemporal[0] = BuffReceive[1];
       cadenaTemporal[1] = BuffReceive[2];
       cadenaTemporal[2] = BuffReceive[3];
       cadenaTemporal[3] = BuffReceive[4];
       cadenaTemporal[4] = '\0';
       strcpy(buffer, cadenaTemporal);
#ifdef SETROLL
       sciKp_1 = atoi(buffer);
#endif
#ifdef SETALT
       sciKp_2 = atoi(buffer);
#endif
    }
    if(BuffReceive[0] == 'I')
    {
       cadenaTemporal[0] = BuffReceive[1];
       cadenaTemporal[1] = BuffReceive[2];
       cadenaTemporal[2] = BuffReceive[3];
       cadenaTemporal[3] = BuffReceive[4];
       cadenaTemporal[4] = '\0';
       strcpy(buffer, cadenaTemporal);
#ifdef SETROLL
       sciKi_1 = atoi(buffer);
#endif
#ifdef SETALT
       sciKi_2 = atoi(buffer);
#endif
    }
    if(BuffReceive[0] == 'D')
    {
       cadenaTemporal[0] = BuffReceive[1];
       cadenaTemporal[1] = BuffReceive[2];
       cadenaTemporal[2] = BuffReceive[3];
       cadenaTemporal[3] = BuffReceive[4];
       cadenaTemporal[4] = '\0';
       strcpy(buffer, cadenaTemporal);
#ifdef SETROLL
       sciKd_1 = atoi(buffer);
#endif
#ifdef SETALT
       sciKd_2 = atoi(buffer);
#endif
    }
#ifdef SETPOWER
    if(BuffReceive[0] == 'M')
    {
       cadenaTemporal[0] = BuffReceive[1];
       cadenaTemporal[1] = BuffReceive[2];
       cadenaTemporal[2] = BuffReceive[3];
       cadenaTemporal[3] = BuffReceive[4];
       cadenaTemporal[4] = '\0';
       strcpy(buffer, cadenaTemporal);
       dcBASE = atoi(buffer);
    }
    if(BuffReceive[0] == 'O')
    {
       cadenaTemporal[0] = '0';
       cadenaTemporal[1] = BuffReceive[2];
       cadenaTemporal[2] = BuffReceive[3];
       cadenaTemporal[3] = BuffReceive[4];
       cadenaTemporal[4] = '\0';
       strcpy(buffer, cadenaTemporal);
       if(BuffReceive[1] == '0')
           offset1 = atoi(buffer);
       if(BuffReceive[1] == '1')
           offset2 = atoi(buffer);
    }
#endif
}
/* USER CODE END */
