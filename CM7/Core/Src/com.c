#include "com.h"

/* Private includes */
#include <stdint.h>
#include <stdio.h>
#include <nrf24l01.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <robot_action.pb.h>
#include "motor.h"

/* Private defines */

/**
 * @name NRF register bit numbers
 *
 * Name of particular bits. Used to make code easier to read.
 */
//!@{
#define STATUS_BIT_RX_DR          6
#define FEATURE_BIT_EN_ACK_PAY    1
#define FEATURE_BIT_EN_DPL        2
//!@}

/**
 * @name NRF register masks
 *
 * Are used to mask out wanted bits from registers in the device. Makes code easier to read.
 */
//!@{
#define STATUS_MASK_BIT_RX_DR      0x40       /**< Data ready in RX FIFO interrupt. */
#define STATUS_MASK_BITS_RX_P_NO   0x0E       /**< Data pipe number for payload available for readin from RX_FIFO. */
//!@}

#define PIPE_CONTROLLER 0
#define PIPE_VISION     1


/* Private functions declarations */
static void parse_controller_packet(uint8_t* payload, uint8_t len);


/*
 * Public functions implementations
 */

void COM_Init(SPI_HandleTypeDef* hspi) {
  uint8_t controllerAddress[5]  = {1,2,3,4,5};
  uint8_t visionAddress[5] = {1,2,3,4,6};

  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // Set the RF channel frequency, it's defined as: 2400 + NRF_REG_RF_CH [MHz]
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x0F);

  // Enable pipe 0 (controller packets) and pipe 1 (ssl vision packets)
  NRF_WriteRegisterByte(NRF_REG_EN_RXADDR, 0x03);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, controllerAddress, 5);
  //NRF_WriteRegister(NRF_REG_RX_ADDR_P1, visionAddress, 5);

  // We enable ACK payloads which needs dynamic payload to function.
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_BIT_EN_ACK_PAY);
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_BIT_EN_DPL);
  NRF_WriteRegisterByte(NRF_REG_DYNPD, 0x03);

  NRF_EnterMode(NRF_MODE_RX);
  printf("[COM] Entered RX mode...\r\n");
}

void COM_RF_HandleIRQ() {
  uint8_t status = NRF_ReadStatus();
  if (status & STATUS_MASK_BIT_RX_DR) {
    uint8_t pipe = (status & STATUS_MASK_BITS_RX_P_NO) >> 1;
    COM_RF_Receive(pipe);
  }
}

void COM_RF_Receive(uint8_t pipe) {
  uint8_t len = 0;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);

  uint8_t payload[len];
  NRF_ReadPayload(payload, len);

  //printf("[COM] Payload of length %i from pipe %i\r\n", len, pipe);

  switch (pipe) {
    case PIPE_CONTROLLER:
      parse_controller_packet(payload, len);
      break;
    case PIPE_VISION:
      break;
  }

  // What we're sending back on next receive
  //uint8_t txMsg = 'W';
  //NRF_WriteAckPayload(pipe, &txMsg, 1);

  NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_BIT_RX_DR);
}

void COM_RF_PrintInfo() {
  NRF_PrintFIFOStatus();
  NRF_PrintStatus();
}


/*
 * Private function implementations
 */

static void parse_controller_packet(uint8_t* payload, uint8_t len) {
  action_Command cmd = action_Command_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(payload, len);
  bool status = pb_decode(&stream, action_Command_fields, &cmd);
  if (!status) {
    printf("[COM] Decoding PB failed: %s\r\n", PB_GET_ERROR(&stream));
    return;
  }

  printf("[COM] robot %d should ", cmd.robot_id);
  switch(cmd.command_id) {
    case action_ActionType_STOP_ACTION:
      printf("STOP");
      break;
    case action_ActionType_KICK_ACTION:
      printf("KICK");
      break;
    case action_ActionType_MOVE_ACTION:
      printf("MOVE");
      break;
    case action_ActionType_INIT_ACTION:
      printf("INIT");
      break;
    case action_ActionType_SET_NAVIGATION_DIRECTION_ACTION:
      printf("NAV");
      break;
    case action_ActionType_ROTATE_ACTION:
      printf("ROTATE");
      break;
  }
  printf("\r\n");
}
