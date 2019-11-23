/*
 * main.c
 *      Author: Dunajski
 */
#include "communication.h"
#include "peripherals.h"
#include "random.h"

int main(void)
{
  InitUsart();
  InitAdc();
  InitTimer0();
  InitTimer1();
  InitTimer2();
  InitIos();
  InitExternalInterupt1();
  sei();

  while (1)
  {
    // zaczynamy od pojscia spac i oczekujemy wybudzenia trzykrotnym nacisnieciem dzwigni
    if (device_state == ST_ENERGY_SAVING)
      GoToSleep();
  }

  return 0;
}
