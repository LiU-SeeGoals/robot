#include "log.h"

/* Privat includes */
#include <stdio.h>
#include <stdarg.h>

/* Private defines */
#define LOG_MODULES_AVAIL 10
#define MODULE_NAME log
#define LOG_BUFFER_SIZE 256

/* Private variables */
static LOG_Module internal_log_mod;
static UART_HandleTypeDef *huart;
static LOG_Module* log_modules[LOG_MODULES_AVAIL];
static int modules_pointer = 0;

/*
 * Public function implementations
 */
void LOG_Init(UART_HandleTypeDef *handle) {
  huart = handle; 
  uint8_t newline[4] = {'\r', '\n', '\r', '\n'};
  HAL_UART_Transmit(huart, newline, 4, HAL_MAX_DELAY);
  LOG_InitModule(&internal_log_mod, "LOG", LOG_LEVEL_INFO);
  LOG_INFO("Initialised...\r\n");
}

void LOG_InitModule(LOG_Module *mod, const char* name, LOG_Level min_out_level) {
  mod->index = modules_pointer;
  mod->min_output_level = min_out_level;
  mod->name = name;
  mod->muted = 0;
  LOG_DEBUG("Adding module: %s\r\n", mod->name);
  log_modules[modules_pointer] = mod;
  modules_pointer++;
}

void LOG_Printf(LOG_Module *mod, LOG_Level level, const char* format, ...) {
  if (level >= mod->min_output_level && !mod->muted) {
    char buffer[LOG_BUFFER_SIZE];
    int offset = 0;

    if (level != LOG_LEVEL_UI) {
      offset += snprintf(buffer, LOG_BUFFER_SIZE, "[%s-%s] ", mod->name, LOG_LEVEL[level].short_name);
    }

    if (offset < LOG_BUFFER_SIZE - 1) {
      va_list args;
      va_start(args, format);
      vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, format, args);
      va_end(args);
    }

    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
  }
}

LOG_Module** LOG_GetModules(int *len) {
  *len = modules_pointer;
  return log_modules;
}

LOG_Module* LOG_GetModule(int index) {
  if (index < 0 || index > modules_pointer) {
    return NULL;
  }
  return log_modules[index];
}
