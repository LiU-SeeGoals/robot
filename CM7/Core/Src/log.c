#include "log.h"

/* Privat includes */
#include <stdio.h>
#include <stdarg.h>

/* Private defines */
#define LOG_BACKENDS_AVAIL 2
#define LOG_MODULES_AVAIL 10
#define MODULE_NAME log
#define LOG_BACKEND_UART 0
#define LOG_BACKEND_BUFFER 1

/* Private variables */
static LOG_Module internal_log_mod;
static UART_HandleTypeDef *huart;
static LOG_Module* log_modules[LOG_MODULES_AVAIL];
static int modules_pointer = 0;
static char log_buffer[LOG_BUFFER_SIZE][LOG_MSG_SIZE];
static int log_buffer_pointer = 0;
static uint8_t cycled_logs = 0;
static LOG_Backend backends[LOG_BACKENDS_AVAIL] = {
  {
    .name = "UART",
    .min_output_level = LOG_LEVEL_UI,
    .muted = 0
  },
  {
    .name = "Buffer",
    .min_output_level = LOG_LEVEL_TRACE,
    .muted = 0
  },
};

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
  mod->min_output_level = min_out_level;
  mod->name = name;
  mod->muted = 0;
  LOG_DEBUG("Adding module: %s\r\n", mod->name);
  log_modules[modules_pointer] = mod;
  modules_pointer++;
}

void LOG_Printf(LOG_Module *mod, LOG_Level msg_level, const char* format, ...) {
  if (msg_level >= mod->min_output_level && !mod->muted) {
    int offset = 0;
    char msg_buffer[LOG_MSG_SIZE];

    if (msg_level != LOG_LEVEL_UI) {
      offset += snprintf(msg_buffer, LOG_MSG_SIZE, "[%s-%s] ", mod->name, LOG_LEVEL[msg_level].short_name);
    }

    if (offset < LOG_MSG_SIZE - 1) {
      va_list args;
      va_start(args, format);
      vsnprintf(msg_buffer + offset, LOG_MSG_SIZE - offset, format, args);
      va_end(args);
    }

    if (msg_level >= backends[LOG_BACKEND_UART].min_output_level &&
        !backends[LOG_BACKEND_UART].muted) {
      HAL_UART_Transmit(huart, (uint8_t*)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
    }

    if (msg_level >= backends[LOG_BACKEND_BUFFER].min_output_level &&
        !backends[LOG_BACKEND_BUFFER].muted) {
      if (cycled_logs) {
        memset(log_buffer[log_buffer_pointer], 0, LOG_MSG_SIZE);
      }
      strncpy(log_buffer[log_buffer_pointer], msg_buffer, strlen(msg_buffer));
      log_buffer_pointer = (log_buffer_pointer + 1) % LOG_BUFFER_SIZE;
      cycled_logs = cycled_logs || log_buffer_pointer == 0;
    }
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

LOG_Backend* LOG_GetBackends(int *len) {
  *len = LOG_BACKENDS_AVAIL;
  return &backends[0];
}

LOG_Backend* LOG_GetBackend(int index) {
  if (index < 0 || index >= LOG_BACKENDS_AVAIL) {
    return NULL;
  }
  return &backends[index];
}

char (*LOG_GetBuffer())[LOG_MSG_SIZE] {
  return log_buffer;
}
