/*
 * main.c
 *      Author: Dunajski
 *      thesis
 */
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "communication.h"
#include "peripherals.h"
#include "random.h"

int main(void)
{
  InitUart();
  InitAdc();
  InitTimer0();
  InitTimer2();
  InitIO();
  sei();
  while (1)
  {
    // zaczynamy od pojscia spac i oczekujemy wybudzenia trzykrotnym nacisnieciem dzwigni
    if(device_state == ST_IDLE)
      GoToSleep();
  }

  return 0;
}
