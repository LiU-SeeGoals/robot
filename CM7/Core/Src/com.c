#include <com.h>

void RF_Init(SPI_HandleTypeDef hspi) {
  uint8_t address[5] = {1,2,3,4,5};
  NRF_Init(&hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    printf("[RF] Couldn't verify nRF24...\r\n");
  }

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // Set the RF channel frequency, it's defined as: 2400 + NRF_REG_RF_CH [MHz]
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x0F);

  // Setup addresses to all pipe 0
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, address, 5);

  // Enable all pipes (so we can receive on them all)
  NRF_WriteRegisterByte(NRF_REG_EN_RXADDR, 0x3F);

  /* To enable ACK payloads we need to setup dynamic payload length. */

  // Enables us to send custom payload with ACKs.
  NRF_SetRegisterBit(NRF_REG_FEATURE, 1);

  // Enables dynamic payload length generally.
  NRF_SetRegisterBit(NRF_REG_FEATURE, 2);

  // Enable dynamic payload lengths on all data pipes.
  // If we didn't do this we'd have to set the payload width
  // that we were going to use for all pipes.
  NRF_WriteRegisterByte(NRF_REG_DYNPD, 0x3F);

  // Enter RX mode and wait for packets.
  // When we get a package NRF_IRQ will be set low,
  // which we handle in the interrupt callback below.
  NRF_EnterMode(NRF_MODE_RX);
  printf("[RF] Entered RX mode...\r\n");
}

void RF_HandleIRQ() {
  // NRF_IRQ was pulled, check whether the status
  // register is saying we've received a package.
  uint8_t status = NRF_ReadStatus();
  if (status & 0x40) {
    // RX_DR is set in register (Data Ready RX FIFO interrupt bit)

    // Read what pipe received this package.
    uint8_t pipe = (status & 0x0E) >> 1;
    RF_Receive(pipe);
  }
}

// Handle the package
void RF_Receive(uint8_t pipe) {
  // Since we have dynamic payload width we'll have to
  // check what the length of the last received package is (which
  // we're currently handling).
  uint8_t length = 0x00;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &length, 1);

  // Once we have the length we can read the payload.
  uint8_t payload[length];
  NRF_ReadPayload(payload, length);

  printf("Payload of length %i from pipe %i\r\n", length, pipe);

  action_Command message = action_Command_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(payload, sizeof(payload));
  bool status = pb_decode(&stream, action_Command_fields, &message);
  if (!status) {
    printf("[RF] Decoding PB failed: %s\r\n", PB_GET_ERROR(&stream));
  } else {
    printf("robot: %d, cmd: %d\r\n", message.robot_id, message.command_id);
  }

  uint8_t msg = 'W';
  // Send something back on next receive
  NRF_WriteAckPayload(pipe, &msg, 1);

  // Reset the RX_DR bit so we can receive
  // new packages.
  NRF_SetRegisterBit(NRF_REG_STATUS, 6);
}

void RF_PrintInfo() {
  NRF_PrintFIFOStatus();
  NRF_PrintStatus();
}
