#include "kicker.h"

/* Private includes */
// ...

/* Private defines */
// ...

/* Private variables */
int max_charges_per_kick = 6;
int charge_wait_us = 25000;
int discharge_wait_us = 130;
int charges_since_last_kick = 0;

/* Private functions declarations */
static void wait(uint64_t us);

/*
 * Public functions implementations
 */

void KICKER_Charge() {
  if (charges_since_last_kick >= max_charges_per_kick) {
    printf("[KICKER] I won't charge more until you KICK ME!\r\n");
    return;
  }

  HAL_GPIO_WritePin(KICKER_CHARGE_GPIO_Port, KICKER_CHARGE_Pin, GPIO_PIN_SET);
  wait(charge_wait_us); // TODO replace with hardware timer
  HAL_GPIO_WritePin(KICKER_CHARGE_GPIO_Port, KICKER_CHARGE_Pin, GPIO_PIN_RESET);
  charges_since_last_kick++;
  printf("[KICKER] Charged %d times...\r\n", charges_since_last_kick);
}

void KICKER_Kick() {
  HAL_GPIO_WritePin(KICKER_DISCHARGE_GPIO_Port, KICKER_DISCHARGE_Pin, GPIO_PIN_RESET);
  wait(discharge_wait_us); // TODO replace with hardware timer
  HAL_GPIO_WritePin(KICKER_DISCHARGE_GPIO_Port, KICKER_DISCHARGE_Pin, GPIO_PIN_SET);
  charges_since_last_kick = 0;
  printf("[KICKER] GOAL!\r\n");
}

void KICKER_EditValue(KICKER_Value what, int val) {
  switch (what) {
    case KICKER_VAL_MAX_CHARGES_PER_KICK:
      max_charges_per_kick = val;
      break;
    case KICKER_VAL_CHARGE_WAIT_US:
      charge_wait_us = val;
      break;
    case KICKER_VAL_DISCHARGE_WAIT_US:
      discharge_wait_us = val;
      break;
  }
}

void KICKER_PrintValues() {
  printf("Max charges per kick: %i\r\n", max_charges_per_kick);
  printf("Charge wait (us): %i\r\n", charge_wait_us);
  printf("Discharge wait (us): %i\r\n", discharge_wait_us);
}

/*
 * Private functions implementations
 */
void wait(uint64_t us) {
  uint32_t volatile cycles = HAL_RCC_GetSysClockFreq() * us / 1000000;
  uint32_t volatile current = 0;
  while (current <= cycles) {
    current++;
  }
}
