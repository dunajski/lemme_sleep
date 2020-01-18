/*
 * main.c
 *      Author: Dunajski
 */
#define F_CPU 8000000

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

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
    MOTOR_ON;
    _delay_ms(5000);
    STATE_LED_OFF;
    MOTOR_OFF;
    _delay_ms(5000);
  }

  while (1)
  {
    // zaczynamy od pojscia spac i oczekujemy wybudzenia trzykrotnym nacisnieciem dzwigni
    if (device_state == ST_ENERGY_SAVING)
      GoToSleep();
  }

  return 0;
}
