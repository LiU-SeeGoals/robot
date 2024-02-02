#include "log.h"

/* Privat includes */
#include <stdio.h>
#include <stdarg.h>

/* Private defines */
#define MODULE_NAME log
#define LOG_BUFFER_SIZE 256
#define LOG_MODULES_AVAIL 10

/* Private variables */
static LOG_Module internal_log_mod;
static LOG_Level stdout_log_level = LOG_LEVEL_DEBUG;
static UART_HandleTypeDef *huart;
static uint8_t level_readable[9] = {'T', 'D', 'I', 'N', 'W', 'E', 'C', 'A', 'E'};
static LOG_Module* log_modules[LOG_MODULES_AVAIL];
static int modules_pointer = 0;

/*
 * Public function implementations
 */
void LOG_Init(UART_HandleTypeDef *handle) {
  huart = handle; 
  uint8_t newline[4] = {'\r', '\n', '\r', '\n'};
  HAL_UART_Transmit(huart, newline, 4, HAL_MAX_DELAY);
  LOG_InitModule(&internal_log_mod, "LOG");
  LOG_INFO("Initialised...\r\n");
}

void LOG_InitModule(LOG_Module *mod, const char* name) {
  mod->index = modules_pointer;
  mod->minimum_output_level = stdout_log_level;
  mod->name = name;
  mod->muted = 0;
  LOG_DEBUG("Adding module: %s\r\n", mod->name);
  log_modules[modules_pointer] = mod;
  modules_pointer++;
}

void LOG_Printf(LOG_Module *mod, LOG_Level level, const char* format, ...) {
  if (level >= mod->minimum_output_level && !mod->muted) {
    char buffer[LOG_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(buffer, LOG_BUFFER_SIZE, "[%s-%c] ", mod->name, level_readable[level]);

    if (offset < LOG_BUFFER_SIZE - 1) {
      va_list args;
      va_start(args, format);
      vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, format, args);
      va_end(args);
    }

    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
  }
}

void LOG_PrintModules() {
  for (int i = 0; i < modules_pointer; i++) {
    LOG_Printf(&internal_log_mod, LOG_LEVEL_INFO, "[%i] %s\r\n", log_modules[i]->index, log_modules[i]->name);
  }
}

void LOG_Module_ToggleMuted(int index) {
  if (index < modules_pointer) {
    log_modules[index]->muted = !log_modules[index]->muted;
    LOG_INFO("Toggle mute: %s\r\n", log_modules[index]->name);
  } else {
    LOG_WARNING("No module with index %i\r\n", index);
  }
}
