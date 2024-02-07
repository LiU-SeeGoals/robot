#include "com.h"

/* Private includes */
#include <stdint.h>
#include <stdio.h>
#include <nrf24l01.h>
#include <nrf_helper_defines.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <robot_action.pb.h>
#include "nav.h"

/* Private defines */
#define PIPE_ACTION     0
#define PIPE_DATA       1
#define PIPE_PING       2

#define CONNECT_MAGIC 0x4d, 0xf8, 0x42, 0x79

#define CONTROLLER_ADDR {2, 255, 255, 255, 255}
#define ROBOT_DATA_ADDR(id) {1, 255, 255, 255, id}
#define ROBOT_ACTION_ADDR(id) {1, 255, 255, id, 255}
#define ROBOT_PING_ADDR(id) {1, 255, id, 255, 255}

/* Private functions declarations */
static void parse_controller_packet(uint8_t* payload, uint8_t len);
static int find_id();

/*
 * Public functions implementations
 */

void COM_Init(SPI_HandleTypeDef* hspi) {
  int id = find_id();
  uint8_t controllerAddress[5] = CONTROLLER_ADDR;
  uint8_t actionAdress[5] = ROBOT_ACTION_ADDR(id);
  uint8_t dataAddress[5] = ROBOT_DATA_ADDR(id);
  uint8_t pingAdress[5] = ROBOT_PING_ADDR(id);
  // Initialize and enter standby-I mode
  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    printf("[RF] Couldn't verify nRF24 SPI...\r\n");
    return;
  }

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // Set the RF channel frequency, it's defined as: 2400 + NRF_REG_RF_CH [MHz]
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x0F);

  // Setup the TX address.
  // We also have to set pipe 0 to receive on the same address.
  NRF_WriteRegister(NRF_REG_TX_ADDR, controllerAddress, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, actionAdress, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, actionAdress, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P2, actionAdress, 5);

  // We enable ACK payloads which needs dynamic payload to function.
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_ACK_PAY);
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_DPL);
  NRF_WriteRegisterByte(NRF_REG_DYNPD, 0x03);

  // Setup for 3 max retries when sending and 500 us between each retry.
  // For motivation, see page 60 in datasheet.
  NRF_WriteRegisterByte(NRF_REG_SETUP_RETR, 0x13);

  
  if (id >= 0) {
    uint8_t data[] = {CONNECT_MAGIC, id};
    if (NRF_Transmit(data, 5) != NRF_OK) {
      printf("[COM] Failed sending ID...\r\n");
    } else {
      printf("[COM] Sent ID...\r\n");
    }
  } else {
    printf("[COM] Bad ID...\r\n");
  }

  NRF_EnterMode(NRF_MODE_RX);
  printf("[COM] Entered RX mode...\r\n");
}

void COM_RF_HandleIRQ() {
  uint8_t status = NRF_ReadStatus();

  if (status & STATUS_MASK_MAX_RT) {
    // Max retries while sending.
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_MAX_RT);
  }

  if (status & STATUS_MASK_TX_DS) {
    // ACK received
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_TX_DS);
  }

  if (status & STATUS_MASK_RX_DR) {
    // Received packet
    uint8_t pipe = (status & STATUS_MASK_RX_P_NO) >> 1;
    COM_RF_Receive(pipe);
  }
}

void COM_RF_Receive(uint8_t pipe) {
  uint8_t len = 0;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);

  uint8_t payload[len];
  NRF_ReadPayload(payload, len);

  printf("[COM] Payload of length %i from pipe %i\r\n", len, pipe);
  /*for (int i = 0; i < len; ++i) {
      printf("%u,", payload[i]);
  }
  printf("\n");*/

  switch (pipe) {
    case PIPE_ACTION:
      parse_controller_packet(payload, len);
      break;
    case PIPE_DATA:
      break;
    case PIPE_PING:
        printf("Got ping!\n");
      break;
  }

  // What we're sending back on next receive
  //uint8_t txMsg = 'W';
  //NRF_WriteAckPayload(pipe, &txMsg, 1);

  NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_RX_DR);
}

void COM_RF_PrintInfo() {
  NRF_PrintFIFOStatus();
  NRF_PrintStatus();
}


/*
 * Private function implementations
 */

// Maps the 96 bit Unique device identifier into a robot id 0-15.
// Returns -1 if this device has no mapping.
static int find_id() {
  uint32_t w0 = HAL_GetUIDw0();
  uint32_t w1 = HAL_GetUIDw1();
  uint32_t w2 = HAL_GetUIDw2();
  if (w0 == 2687023 && w1 == 858935561 && w2 == 808727605) {
    return 0;
  }

  printf("Unmapped id: %iu, %iu, %iu\n", w0, w1, w2);
  return -1;
}

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
      NAV_Stop();
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
      {
        printf("NAV ");
        switch(cmd.direction.x) {
          case 1: // left
            printf("left");
            NAV_Direction(LEFT);
            break;
          case -1: // right
            printf("right");
            NAV_Direction(RIGHT);
            break;
        }

        switch(cmd.direction.y) {
          case 1: // up
            printf("up");
            NAV_Direction(UP);
            break;
          case -1: // down
            printf("down");
            NAV_Direction(DOWN);
            break;
        }
      }
      break;
    case action_ActionType_ROTATE_ACTION:
      printf("ROTATE");
      break;
  }
  printf("\r\n");
}
