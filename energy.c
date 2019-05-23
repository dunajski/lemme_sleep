/*
 * energy.c
 *
 *  Created on: 23 maj 2019
 *      Author: Dunajski
 */
#include <avr/interrupt.h>
#include "random.h"

// przerawanie od externala do wybudzania micro
ISR(INT1_vect)
{
  if (device_state == ST_IDLE)
  {
    // zapisz ze nacisniete i w przerwaniu od timera dodaj ilosc wcisniec
  }
}

