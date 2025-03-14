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
#define CONNECT_MAGIC   0x4d, 0xf8, 0x42, 0x79
#define CONTROLLER_ADDR {2, 255, 255, 255, 255}
#define ROBOT_ACTION_ADDR(id) {1, 255, 255, id, 255}
#define RF_CONTROLLER_PIPE 0

/* Private functions declarations */
static void parse_controller_packet(uint8_t* payload, uint8_t len);
static char* ping_ack_to_string(uint8_t ack);

/* Private variables */
static LOG_Module internal_log_mod;
static int nRFon = 0;
static volatile uint8_t ping_ack;
volatile uint8_t last_rec_id = 0xff;
volatile uint32_t last_rec_time = 0;

/*
 * Public functions implementations
 */

void COM_Init(SPI_HandleTypeDef* hspi, uint8_t* nrf_available) {
  LOG_InitModule(&internal_log_mod, "COM", LOG_LEVEL_INFO, 0);

  // Initialize and enter standby-I mode
  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    LOG_ERROR("Couldn't verify nRF24 SPI communication...\r\n");
    return;
  } else {
    *nrf_available = 1;
  }

  nRFon = 1;

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // Setup the nRF registers
  COM_RF_Init();
}

void COM_RF_Init() {
  int id = COM_Get_ID();
  uint8_t controllerAddress[5]  = CONTROLLER_ADDR;
  uint8_t actionAdress[5] = ROBOT_ACTION_ADDR(id);

  // See nRF24L01+ chapter 6.3 for more...
  // Set the RF channel frequency to 2500, i.e. outside of wifi range
  // It's defined as: 2400 + NRF_REG_RF_CH [MHz]
  // NRF_REG_RF_CH can 0-127, but not all values seem to work.
  // 2525 and below works, 2527 had issues...
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x7d); // 2525
  //NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x64); // 2500

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
  LOG_DEBUG("Initialized RF...\r\n");
}

void COM_RF_HandleIRQ() {
  uint8_t status = NRF_ReadStatus();

  if (status & STATUS_MASK_RX_DR) {
    // Received packet
    for (;;) {
      uint8_t pipe = (NRF_ReadStatus() & STATUS_MASK_RX_P_NO) >> 1;
      if (pipe >= 6) {
        break;
      }
      COM_RF_Receive(pipe);
    }
  }

  if (status & STATUS_MASK_TX_DS) {
    // ACK received
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_TX_DS);
    ping_ack = 1;
  }

  if (status & STATUS_MASK_MAX_RT) {
    // Max retries while sending.
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_MAX_RT);
    ping_ack = 2;
  }

  if (!status) {
    LOG_ERROR("RF_HandleIRQ called but status empty...\r\n");
  }
}

void COM_RF_Receive(uint8_t pipe) {
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);

  uint8_t len = 0;
  NRF_Status status;
  status = NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);
  if (status != NRF_OK) {
    LOG_ERROR("Couldn't read length of RF packet...\r\n");
  }

  //if (len == 0 || pipe == RF_CONTROLLER_PIPE) {
  //  return;
  //}

  uint8_t payload[len];
  status = NRF_ReadPayload(payload, len);
  if (status != NRF_OK) {
    LOG_ERROR("Couldn't read RF packet payload...\r\n");
  }

  NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_RX_DR);

  uint8_t msg_type = payload[0] & 0x0f;
  uint8_t order = payload[0] & 0xf0;
  last_rec_time = HAL_GetTick(); // Timestamp of last received message
  if (order == last_rec_id) {
    return;
  }

  /* The id of last message.
   * The value is contained in the upper 4 bits. Sequential ids should be: 0x00, 0x10, 0x20, etc.
   * 0xff is used for no last message, 0xfe for connection timed out.
   * This field is used to remove duplicate messages. */
  last_rec_id = order;

  switch (msg_type) {
    case MSG_ACTION:
      parse_controller_packet(payload + 1, len - 1);
      break;
    case MSG_PING:
      main_tasks |= TASK_PING;
      break;
    default:
      LOG_WARNING("Unkown message type %d\r\n", msg_type);
      HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
      break;
  }

  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
  NRF_SendCommand(NRF_CMD_FLUSH_RX);
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

void COM_RF_Reset() {
  // Initialize and enter standby-I mode
  if(NRF_VerifySPI() != NRF_OK) {
    LOG_ERROR("Couldn't verify nRF24 SPI communication...\r\n");
    return;
  } else {
    LOG_INFO("Could communicate with nRF24.\r\n");
  }

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  LOG_INFO("nRF24 resetted.\r\n");

  COM_RF_Init();
}

void COM_Ping() {
  int id = COM_Get_ID();

  if (id >= 0) {
    HAL_Delay(100);
    NRF_EnterMode(NRF_MODE_STANDBY1);

    ping_ack = 0;
    uint8_t data[] = {CONNECT_MAGIC, id};
    if (NRF_Transmit(data, 5) != NRF_OK) {
      LOG_INFO("Ping: Failed sending ID...\r\n");
    } else {
      LOG_INFO("Ping: Sent ID %d to basestation...\r\n", id);
    }
    uint32_t stamp = HAL_GetTick();
    while (!ping_ack && HAL_GetTick() - stamp < 1000) {
      HAL_Delay(1);
    }
    if (ping_ack != 1) {
      NRF_SendCommand(NRF_CMD_FLUSH_TX);
    } else {
      last_rec_time = HAL_GetTick();
    }

    LOG_INFO("Ping: Got ack {%s}\r\n", ping_ack_to_string(ping_ack));
  } else {
    LOG_INFO("Ping: Bad ID: (%u, %u, %u)\r\n", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
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

void COM_RF_Send(uint8_t *msg, uint8_t length) {
  NRF_EnterMode(NRF_MODE_STANDBY1);

  if (NRF_TransmitAndWait(msg, length) != NRF_OK) {
    LOG_ERROR("Failed sending message...\r\n");
  }

  NRF_EnterMode(NRF_MODE_RX);
}

uint8_t COM_Get_ID() {
  uint32_t w0 = HAL_GetUIDw0();
  uint32_t w1 = HAL_GetUIDw1();
  uint32_t w2 = HAL_GetUIDw2();

  if (w0 == 3211302 && w1 == 892490001 && w2 == 842217265)
  {
    return 0;
  }
  if (w0 == 3080253 && w1 == 892490001 && w2 == 842217265)
  {
    return 2;
  }
  if (w0 == 3932237 && w1 == 892490001 && w2 == 842217265)
  {
    return 1;
  }
  if (w0 == 2490418 && w1 == 858935561 && w2 == 808727605)
  {
    return 3;
  }
  if (w0 == 4259883  && w1 == 892490001 && w2 == 842217265)
  {
    return 4;
  }
  if (w0 == 4522020 && w1 == 892490001 && w2 == 842217265)
  {
    return 5;
  }
  if (w0 == 2687023 && w1 == 858935561 && w2 == 808727605)
  {
    return 6;
  }
  if (w0 == 2293800 && w1 == 858935561 && w2 == 808727605)
  {
    return 0;
  }
  LOG_ERROR("Failed ID lookup for robot ID: %d %d %d\r\n", w0, w1, w2);
  return 255;
}

/*
 * Private function implementations
 */

static void parse_controller_packet(uint8_t* payload, uint8_t len) {
  Command* cmd = NULL;
  cmd = command__unpack(NULL, len, payload);
  if (!cmd) {
    LOG_WARNING("Decoding PB failed\r\n");
    return;
  }
  NAV_HandleCommand(cmd);
  protobuf_c_message_free_unpacked((ProtobufCMessage*) cmd, NULL);
}

static char* ping_ack_to_string(uint8_t ack) {
    switch (ack) {
        case 0: return "Unknown transmission error";
        case 1: return "ACK received";
        case 2: return "Max retries reached while sending";
        default: return "Invalid ACK code";
    }
}
