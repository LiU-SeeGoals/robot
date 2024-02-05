#ifndef KICKER_H
#define KICKER_H

#include "main.h"

typedef struct {
  int max_charges_per_kick;
  int charge_wait_us;
  int discharge_wait_us;
  int charges_since_last_kick;
} KICKER_Settings;

void KICKER_Init();
void KICKER_Charge();
void KICKER_Kick();
KICKER_Settings* KICKER_GetSettings();

#endif /* KICKER_H */
