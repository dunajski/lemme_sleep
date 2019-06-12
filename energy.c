/*
 * energy.c
 *
 *  Created on: 23 maj 2019
 *      Author: Dunajski
 */
#include <avr/interrupt.h>
#include "random.h"
#include "energy.h"

volatile uint16 debounce_idle_delay = 0;
// przerawanie od externala do wybudzania micro
ISR(INT1_vect)
{
  static uint8 pressed_times = 0;

  if (debounce_idle_delay == 0)
  {
    if (device_state == ST_IDLE)
    {
      // zapisz ze nacisniete i w przerwaniu od timera dodaj ilosc wcisniec
      pressed_times++;
      debounce_idle_delay = ISR_DEBOUNCE_CNT;
    }
  }

  // jesli ktos wcisnie trzykrotnie przechodzi w stan losowania
  if (pressed_times >= 3)
  {
    pressed_times = 0;
    device_state = ST_LOSOWANIE;
    DEBUG_LED_OFF;
    #if DEBUG_STATE == _ON
    StrToSerial("Wstaje, losuje\n");
    #endif
  }
}

