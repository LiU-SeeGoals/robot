#include "log.h"

/* Privat includes */
#include <stdio.h>
#include <stdarg.h>

/* Private defines */
#define LOG_BUFFER_SIZE 256

/* Private variables */
static LOG_Level stdout_log_level = LOG_LEVEL_DEBUG;
static UART_HandleTypeDef *huart;

/*
 * Public function implementations
 */
void LOG_Init(UART_HandleTypeDef *handle) {
  huart = handle; 
  uint8_t newline[4] = {'\r', '\n', '\r', '\n'};
  HAL_UART_Transmit(huart, newline, 4, HAL_MAX_DELAY);
}

void LOG_InitModule(LOG_Module *mod, const char* name) {
  mod->level = stdout_log_level;
  mod->name = name;
}

void LOG_Printf(LOG_Module *mod, LOG_Level level, const char* format, ...) {
  if (level >= stdout_log_level) {
    char buffer[LOG_BUFFER_SIZE];
    int offset = 0;
    offset += snprintf(buffer, LOG_BUFFER_SIZE, "[%s-%i] ", mod->name, level);

    if (offset < LOG_BUFFER_SIZE - 1) {
      va_list args;
      va_start(args, format);
      vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, format, args);
      va_end(args);
    }

    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
  }
}
