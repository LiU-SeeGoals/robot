#ifndef LOG_H
#define LOG_H

/* Public includes */
#include "main.h"

/* Public defines */
#define LOG_BUFFER_SIZE 5
#define LOG_MSG_SIZE    100

#define LOG_PRINTF(level, fmt, ...) LOG_Printf(&internal_log_mod, level, fmt, ##__VA_ARGS__)
#define LOG_UI(fmt, ...)            LOG_Printf(&internal_log_mod, LOG_LEVEL_UI, fmt, ##__VA_ARGS__)
#define LOG_TRACE(fmt, ...)         LOG_Printf(&internal_log_mod, LOG_LEVEL_TRACE, fmt, ##__VA_ARGS__)
#define LOG_DEBUG2(fmt, ...)        LOG_Printf(&internal_log_mod, LOG_LEVEL_DEBUG2, fmt, ##__VA_ARGS__)
#define LOG_DEBUG1(fmt, ...)        LOG_Printf(&internal_log_mod, LOG_LEVEL_DEBUG1, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)         LOG_Printf(&internal_log_mod, LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)          LOG_Printf(&internal_log_mod, LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_NOTICE(fmt, ...)        LOG_Printf(&internal_log_mod, LOG_LEVEL_NOTICE, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)       LOG_Printf(&internal_log_mod, LOG_LEVEL_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)         LOG_Printf(&internal_log_mod, LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOG_CRITICAL(fmt, ...)      LOG_Printf(&internal_log_mod, LOG_LEVEL_CRITICAL, fmt, ##__VA_ARGS__)
#define LOG_ALERT(fmt, ...)         LOG_Printf(&internal_log_mod, LOG_LEVEL_ALERT, fmt, ##__VA_ARGS__)
#define LOG_EMERGENCY(fmt, ...)     LOG_Printf(&internal_log_mod, LOG_LEVEL_EMERGENCY, fmt, ##__VA_ARGS__)

/* Public enums */

/**
 * @brief The available log levels, from lowest to highest severity.
 *
 * Descriptions from ChatGPT, thanks!
 */
typedef enum {
  /*
   * Output from the CLI UI which is reachable through UART.
   */
  LOG_LEVEL_UI,

  /**
   * Very detailed information, useful for developers to trace program 
   * execution in a granular way. Typically disabled in production due
   * to the high volume of logs generated.
   *
   * Use case: Function entry and exit, detailed state machine behaviors.
   */
  LOG_LEVEL_TRACE,

  /*
   * Informational events useful during development and debugging.
   * Provides insights into the application's behavior under normal
   * operation.
   *
   * Use case: Initialization routines, status of hardware peripherals after
   * setup, algorithmic steps.
   */
  LOG_LEVEL_DEBUG2,
  LOG_LEVEL_DEBUG1,
  LOG_LEVEL_DEBUG,

  /*
   * General operational information about the system's state. It
   * should not be overly verbose and provide a clear overview of the
   * application's status.
   *
   * Use case: System startup, network status, device or sensor readings at
   * a regular interval.
   */
  LOG_LEVEL_INFO,

  /*
   * Normal but significant events that highlight key milestones or
   * transitions in the application's lifecycle.
   *
   * Use case: Mode transitions, completion of significant background tasks,
   * or readiness of critical components.
   */
  LOG_LEVEL_NOTICE,

  /**
   * Indications of potential issues that are not immediate problems
   * but might require attention or could lead to errors if ignored.
   *
   * Use case: Memory usage nearing high watermark, retrying a failed operation,
   * deprecated API usage.
   */
  LOG_LEVEL_WARNING,

  /**
   * Error conditions that are not fatal but indicate failure in specific
   * operations or inability to perform a requested action.
   *
   * Use case: Failed to read from a sensor, communication timeouts, hardware
   * peripheral failures.
   */
  LOG_LEVEL_ERROR,
  /**
   * Critical conditions that require immediate attention. Often related
   * to serious hardware or software failures that could jeopardize the system's stability.
   *
   * Use case: Critical resource depletion, watchdog timer resets, system instability issues.
   */
  LOG_LEVEL_CRITICAL,

  /**
   * A condition that must be fixed immediately to prevent imminent system
   * failure. This level is typically used to highlight issues that escalate quickly.
   *
   * Use case: Overheating detected, critical communication link down, safety
   * mechanism triggered.
   */
  LOG_LEVEL_ALERT,

  /**
   * The highest severity level, indicating the system is unusable
   * or in a state of complete failure, often requiring a system restart or intervention.
   *
   * Use case: System crash, unrecoverable error, complete loss of functionality.
   */
  LOG_LEVEL_EMERGENCY,
} LOG_Level;

/* Public structs */
typedef struct {
  LOG_Level level;
  const char* name;
  const char* short_name;
} LOG_Level_Info;

/* Public variables */
static LOG_Level_Info LOG_LEVEL[LOG_LEVEL_EMERGENCY + 1] = {
  {
    .level = LOG_LEVEL_UI,
    .name = "User Interface",
    .short_name = "UI",
  },
  {
    .level = LOG_LEVEL_TRACE,
    .name = "Trace",
    .short_name = "T",
  },
  {
    .level = LOG_LEVEL_DEBUG2,
    .name = "Debug2",
    .short_name = "D2",
  },
  {
    .level = LOG_LEVEL_DEBUG1,
    .name = "Debug1",
    .short_name = "D1",
  },
  {
    .level = LOG_LEVEL_DEBUG,
    .name = "Debug",
    .short_name = "D",
  },
  {
    .level = LOG_LEVEL_INFO,
    .name = "Info",
    .short_name = "I",
  },
  {
    .level = LOG_LEVEL_NOTICE,
    .name = "Notice",
    .short_name = "N",
  },
  {
    .level = LOG_LEVEL_WARNING,
    .name = "Warning",
    .short_name = "W",
  },
  {
    .level = LOG_LEVEL_ERROR,
    .name = "Error",
    .short_name = "E",
  },
  {
    .level = LOG_LEVEL_CRITICAL,
    .name = "Critical",
    .short_name = "C",
  },
  {
    .level = LOG_LEVEL_ALERT,
    .name = "Alert",
    .short_name = "A",
  },
  {
    .level = LOG_LEVEL_EMERGENCY,
    .name = "Emergency",
    .short_name = "E!",
  }
};

/**
 * Every submodule of the project should have a LOG_Module which give us finer
 * logging controls.
 */
typedef struct {
  LOG_Level min_output_level;
  const char* name;
  uint8_t muted;
} LOG_Module;

/**
 * A backend is someting outputting log messages to somewhere. Examples are
 * through UART or to a local log buffer.
 */
typedef struct {
  LOG_Level min_output_level;
  const char* name;
  uint8_t muted;
} LOG_Backend;

/* Public function declarations */
void LOG_Init(UART_HandleTypeDef *handle);
void LOG_InitModule(LOG_Module *mod, const char* name, LOG_Level min_output_level);
void LOG_Printf(LOG_Module *mod, LOG_Level level, const char* format, ...);
void LOG_PrintLogBuffer(int start, int end);
LOG_Module** LOG_GetModules(int *len);
LOG_Module* LOG_GetModule(int index);
LOG_Backend* LOG_GetBackends(int *len);
LOG_Backend* LOG_GetBackend(int index);
char (*LOG_GetBuffer())[LOG_MSG_SIZE];

#endif /* LOG_H */
