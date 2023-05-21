
#ifndef COMMON_H
#define COMMON_H

#include "MKL25Z4.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>




void set_pwm(uint16_t in);
void TPM0_Init();


#endif