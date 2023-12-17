#ifndef COM_H
#define COM_H

#include <stdint.h>
#include <main.h>
#include <nrf24l01.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <robot_action.pb.h>

/**
 * \brief Configures the NRF device according to this robots serial number.
 */
static void RF_Init(SPI_HandleTypeDef hspi);

/**
 * \brief Parse the received message and handle it correctly.
 * \param pipe What pipe the message was received on.
 */
static void RF_Receive(uint8_t pipe);

static void RF_HandleIRQ();

static void RF_PrintInfo(void);

#endif /* COM_H */
