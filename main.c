/*
 * main.c
 *      Author: Dunajski
 *      thesis
 */
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "communication.h"
#include "peripherals.h"
#include "random.h"

int main(void)
{
  InitUsart();
  InitAdc();
  InitTimer0();
  InitTimer2();
  InitIOs();
  InitExternalInterupt1();
  sei();
  while (1)
  {
    // zaczynamy od pojscia spac i oczekujemy wybudzenia trzykrotnym nacisnieciem dzwigni
    if(device_state == ST_POWER_DWN)
      GoToSleep();
  }

  return 0;
}
