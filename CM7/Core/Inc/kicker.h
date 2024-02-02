#ifndef KICKER_H
#define KICKER_H

#include "main.h"

typedef enum {
  KICKER_VAL_MAX_CHARGES_PER_KICK,
  KICKER_VAL_CHARGE_WAIT_US,
  KICKER_VAL_DISCHARGE_WAIT_US,
} KICKER_Value;

void KICKER_Charge();
void KICKER_Kick();
void KICKER_PrintValues();
void KICKER_EditValue(KICKER_Value what, int val);

#endif /* KICKER_H */
