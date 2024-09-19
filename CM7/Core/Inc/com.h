#ifndef COM_H
#define COM_H

/* Public includes */
#include "main.h"
#include <stdbool.h>

/* Public constants */
#define COM_BASESTATION_TIMEOUT_MS 3000

/* Public function declarations */

/**
 * Configures the NRF device according to this robots serial number.
 *
 * @param hspi The handle for the SPI communication.
 */
void COM_Init(SPI_HandleTypeDef* hspi);

/**
 * Ping the basestation.
 */
void COM_Ping();

/**
 * Checks if a message was received within COM_BASESTATION_TIMEOUT_MS.
 * Call often.
 */
bool COM_Update();

/**
 * Parse the received message and handle it correctly.
 *
 * @param pipe What pipe the message was received on.
 */
void COM_RF_Receive(uint8_t pipe);

/**
 * Handles interrupts sent from the IRQ pin on the NRF.
 */
void COM_RF_HandleIRQ(void);

/**
 * Printf:s status and FIFO status registers from the NRF.
 */
void COM_RF_PrintInfo(void);

#endif /* COM_H */
