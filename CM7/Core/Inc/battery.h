#ifndef BATTERY_H
#define BATTERY_H

/* Public includes */
#include "main.h"

/* Public constants */

/* Public function declarations */
void BATTERY_Init(ADC_HandleTypeDef *hadc);

uint32_t BATTERY_GetLevel();

#endif /* BATTERY_H */
