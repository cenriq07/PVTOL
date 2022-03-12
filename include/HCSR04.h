/*
 * HCSR04.h
 *
 *  Created on: 6 mar 2022
 *      Author: Enrique
 */

#ifndef SOURCE_HCSR04_H_
#define SOURCE_HCSR04_H_

#include "gio.h"
#include "sci.h"
#include "stdio.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "os_task.h"

#define TRIGGER     0
#define ECHO        1
#define US_PORT     gioPORTA

float usRanging();


#endif /* SOURCE_HCSR04_H_ */
