#include "com.h"

/* Private includes */
#include <stdint.h>
#include <stdio.h>
#include <nrf24l01.h>
#include <nrf_helper_defines.h>
#include <robot_action/robot_action.pb-c.h>
#include "main.h"
#include "nav.h"
#include "log.h"

/* Private defines */
#define MSG_PING       0
#define MSG_ACTION     1


#define CONNECT_MAGIC 0x4d, 0xf8, 0x42, 0x79

#define CONTROLLER_ADDR {2, 255, 255, 255, 255}
#define ROBOT_ACTION_ADDR(id) {1, 255, 255, id, 255}


/* Private functions declarations */
static void parse_controller_packet(uint8_t* payload, uint8_t len);
static int find_id();

/* Private variables */
static LOG_Module internal_log_mod;
static int nRFon = 0;

static volatile uint8_t ping_ack;

/*
 * Public functions implementations
 */

void COM_Init(SPI_HandleTypeDef* hspi) {
  int id = find_id();
  uint8_t controllerAddress[5]  = CONTROLLER_ADDR;
  uint8_t actionAdress[5] = ROBOT_ACTION_ADDR(id);

  LOG_InitModule(&internal_log_mod, "COM", LOG_LEVEL_INFO);
  // Initialize and enter standby-I mode
  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    LOG_ERROR("Couldn't verify nRF24 SPI communication...\r\n");
    return;
  }

  nRFon = 1;

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // Set the RF channel frequency, it's defined as: 2400 + NRF_REG_RF_CH [MHz]
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x0F);

  // Setup the TX address.
  // We also have to set pipe 0 to receive on the same address.
  NRF_WriteRegister(NRF_REG_TX_ADDR, controllerAddress, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, controllerAddress, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, actionAdress, 5);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P2, 1);
  NRF_SetRegisterBit(NRF_REG_EN_RXADDR, 0x07);

  // We enable ACK payloads which needs dynamic payload to function.
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_ACK_PAY);
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_DPL);
  NRF_WriteRegisterByte(NRF_REG_DYNPD, 0x07);

  // Setup for 3 max retries when sending and 500 us between each retry.
  // For motivation, see page 60 in datasheet.
  NRF_WriteRegisterByte(NRF_REG_SETUP_RETR, 0x13);

  main_tasks |= TASK_PING;

  NRF_EnterMode(NRF_MODE_RX);
  LOG_INFO("Entered RX mode...\r\n");
}

void COM_RF_HandleIRQ() {
  uint8_t status = NRF_ReadStatus();

  if (status & STATUS_MASK_MAX_RT) {
    // Max retries while sending.
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_MAX_RT);
    ping_ack = 2;
  }

  if (status & STATUS_MASK_TX_DS) {
    // ACK received
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_TX_DS);
    ping_ack = 1;
  }

  if (status & STATUS_MASK_RX_DR) {
    // Received packet
    uint8_t pipe = (status & STATUS_MASK_RX_P_NO) >> 1;
    COM_RF_Receive(pipe);
  }
}

/* The id of last message.
 * The value is contained in the upper 4 bits. Sequential ids should be: 0x00, 0x10, 0x20, etc.
 * 0xff is used for no last message, 0xfe for connection timed out.
 * This field is used to remove duplicate messages.
*/
volatile uint8_t last_rec_id = 0xff;
// Timestamp of last message.
volatile uint32_t last_rec_time = 0;

void COM_RF_Receive(uint8_t pipe) {
  uint8_t len = 0;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);

  uint8_t payload[len];
  NRF_ReadPayload(payload, len);

  NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_RX_DR);


  if (len == 0 || pipe == 0) {
    return;
  }
  main_tasks |= TASK_DATA;
  uint8_t msg_type = payload[0] & 0xf;
  uint8_t order = payload[0] & 0xf0;
  last_rec_time = HAL_GetTick();
  if (order == last_rec_id) {
    return;
  }
  last_rec_id = order;
  //LOG_INFO("Payload of length %i of type %i\r\n", len, msg_type);

  switch (msg_type) {
    case MSG_ACTION:
      parse_controller_packet(payload + 1, len - 1);
      break;
    case MSG_PING:
      main_tasks |= TASK_PING;
      break;
    default:
      LOG_WARNING("Unkown message type %d\r\n", msg_type);
  }

  // What we're sending back on next receive
  //uint8_t txMsg = 'W';
  //NRF_WriteAckPayload(pipe, &txMsg, 1);

}

void COM_RF_PrintInfo(void) {
  uint8_t ret = NRF_ReadStatus();

  if (!nRFon) {
    LOG_INFO("nRF24 not running...\r\n");
    return;
  }

  LOG_INFO("Status register: %02X\r\n", ret);
  LOG_INFO("TX_FULL:  %1X\r\n", ret & (1<<0));
  LOG_INFO("RX_P_NO:  %1X\r\n", (ret & (0x3<<1)) >> 1);
  LOG_INFO("MAX_RT:   %1X\r\n", (ret & (1<<4))    >> 4);
  LOG_INFO("TX_DS:    %1X\r\n", (ret & (1<<5))     >> 5);
  LOG_INFO("RX_DR:    %1X\r\n", (ret & (1<<6))     >> 6);
  LOG_INFO("\r\n");

  ret = NRF_ReadRegisterByte(NRF_REG_FIFO_STATUS);
  LOG_INFO("FIFO status register: %02X\r\n", ret);
  LOG_INFO("RX_EMPTY:   %2X\r\n", ret &  (1<<0));
  LOG_INFO("RX_FULL:    %2X\r\n", (ret & (1<<1)) >> 1);
  LOG_INFO("TX_EMPTY:   %2X\r\n", (ret & (1<<4)) >> 4);
  LOG_INFO("TX_FULL:    %2X\r\n", (ret & (1<<5)) >> 5);
  LOG_INFO("TX_REUSE:   %2X\r\n", (ret & (1<<6)) >> 6);
  LOG_INFO("\r\n");

  ret = NRF_ReadRegisterByte(NRF_REG_CONFIG);
  LOG_INFO("Config register: %02X\r\n", ret);
  LOG_INFO("PRIM_RX:      %1X\r\n", ret & (1<<0));
  LOG_INFO("PWR_UP:       %1X\r\n", ret & (1<<1));
  LOG_INFO("CRCO:         %1X\r\n", ret & (1<<2));
  LOG_INFO("EN_CRC:       %1X\r\n", ret & (1<<3));
  LOG_INFO("MASK_MAX_RT:  %1X\r\n", ret & (1<<4));
  LOG_INFO("MASK_TX_DS:   %1X\r\n", ret & (1<<5));
  LOG_INFO("MASK_RX_DR:   %1X\r\n", ret & (1<<6));
  LOG_INFO("\r\n");
}

void COM_Ping() {
  int id = find_id();

  if (id >= 0) {
    HAL_Delay(100);
    NRF_EnterMode(NRF_MODE_STANDBY1);

    ping_ack = 0;
    uint8_t data[] = {CONNECT_MAGIC, id};
    if (NRF_Transmit(data, 5) != NRF_OK) {
      LOG_INFO("Failed sending ID...\r\n");
    } else {
      LOG_INFO("Sent ID %d...\r\n", id);
    }
    uint32_t stamp = HAL_GetTick();
    while (!ping_ack && HAL_GetTick() - stamp < 1000) {
      HAL_Delay(1);
    }
    if (ping_ack != 1) {
      NRF_SendCommand(NRF_CMD_FLUSH_TX);
    }
    LOG_INFO("Ack ping %d\r\n", ping_ack);
  } else {
    LOG_INFO("Bad ID: (%u, %u, %u)\r\n", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
  }
  NRF_EnterMode(NRF_MODE_RX);
}

bool COM_Update() {
  if (HAL_GetTick() - last_rec_time < COM_BASESTATION_TIMEOUT_MS) {
    return true;
  }
  if (last_rec_id != 0xfe) {
    last_rec_id = 0xfe;
    LOG_INFO("Basestation connection timed out\r\n");
  }
  return false;
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
  if (w0 == 3080253 && w1 == 892490001 && w2 == 842217265) {
    return 1;
  }
  if (w0 == 2490418 && w1 == 858935561 && w2 == 808727605) {
    return 2;
  }
  return -1;
}

static void parse_controller_packet(uint8_t* payload, uint8_t len) {
  Command* cmd = NULL;
  cmd = command__unpack(NULL, len, payload);

  if (!cmd) {
    LOG_WARNING("Decoding PB failed\r\n");
    return;
  }
  NAV_QueueCommandIRQ(cmd);
  main_tasks |= TASK_NAV_COMMAND;
}



