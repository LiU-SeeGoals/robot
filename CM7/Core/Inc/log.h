#ifndef LOG_H
#define LOG_H

/* Public includes */
#include "main.h"

/* Public enums */

/**
 * @brief The available log levels, from lowest to highest severity.
 *
 * Descriptions from ChatGPT, thanks!
 */
typedef enum {
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
} LOG_Module;

/* Public function declarations */
void LOG_Init(UART_HandleTypeDef *handle);
void LOG_InitModule(LOG_Module *mod, const char* name);
void LOG_Printf(LOG_Module *mod, LOG_Level level, const char* format, ...);


#endif /* LOG_H */
