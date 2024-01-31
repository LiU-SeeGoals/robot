#include "kicker.h"

/* Private includes */
// ...

/* Private defines */
#define MAX_CHARGES_PER_KICK  6 // TODO test this value!
#define CHARGE_WAIT_US        25000
#define DISCHARGE_WAIT_US     130

/* Private variables */
int charges_since_last_kick = 0;

/* Private functions declarations */
static void wait(uint64_t us);


/*
 * Public functions implementations
 */

void KICKER_Charge() {
  if (charges_since_last_kick >= MAX_CHARGES_PER_KICK) {
    printf("[KICKER] I won't charge more until you KICK ME!\r\n");
    return;
  }

  HAL_GPIO_WritePin(KICKER_CHARGE_GPIO_Port, KICKER_CHARGE_Pin, GPIO_PIN_SET);
  wait(CHARGE_WAIT_US); // TODO replace with hardware timer
  HAL_GPIO_WritePin(KICKER_CHARGE_GPIO_Port, KICKER_CHARGE_Pin, GPIO_PIN_RESET);
  charges_since_last_kick++;
  printf("[KICKER] Charged %d times...\r\n", charges_since_last_kick);
}

void KICKER_Kick() {
  HAL_GPIO_WritePin(KICKER_DISCHARGE_GPIO_Port, KICKER_DISCHARGE_Pin, GPIO_PIN_RESET);
  wait(DISCHARGE_WAIT_US); // TODO replace with hardware timer
  HAL_GPIO_WritePin(KICKER_DISCHARGE_GPIO_Port, KICKER_DISCHARGE_Pin, GPIO_PIN_SET);
  charges_since_last_kick = 0;
  printf("[KICKER] GOAL!\r\n");
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
