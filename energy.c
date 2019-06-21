/*
 * energy.c
 *
 *  Created on: 23 maj 2019
 *      Author: Dunajski
 */
#include <avr/interrupt.h>
#include "random.h"
#include "energy.h"

/*
 *******************************************************************************
 * Przerwanie od wybudzania MCU pzyciskiem.
 *******************************************************************************
 */
ISR(INT0_vect)
{
  // wylacz, zeby przeciwdzialac wielu wywolaniom, po obsludze przerwania
  // nastepuje uruchomienie wszystkich przerwan oprocz EXT0
  GICR &= ~(1 << INT0);
  // waking up...
  device_state = ST_WAIT_TO_WAKE_UP;
}

