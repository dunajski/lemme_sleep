/*
 * main.c
 *      Author: Dunajski
 */
#include "communication.h"
#include "peripherals.h"
#include "random.h"
#include <util/delay.h>

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

  for (int i = 0; i < 5; i++)
  {
  STATE_LED_ON;
  _delay_ms(500);
  STATE_LED_OFF;
  _delay_ms(500);

  }

  while (1)
  {
    // zaczynamy od pojscia spac i oczekujemy wybudzenia trzykrotnym nacisnieciem dzwigni
    if (device_state == ST_ENERGY_SAVING)
      GoToSleep();
  }

  return 0;
}
