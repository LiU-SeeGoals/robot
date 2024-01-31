#ifndef DEBUG_UI_H
#define DEBUG_UI_H

#include "main.h"

/* Public variables */
// ...

/*
 * Public function 
 */

void DEBUG_UI_Init(UART_HandleTypeDef *huart);
void DEBUG_UI_PrintHelp();
void DEBUG_UI_RxCallback();

#endif /* DEBUG_UI_H */
