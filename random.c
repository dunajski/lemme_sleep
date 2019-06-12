/*
 * random.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#include <avr/interrupt.h>
#include "random.h"

volatile uint16 random_values_grouped[NUM_ACTIONS] = {5000, 5000, 5000, 5000, 5000};
volatile uchar change_random = 0;

//ISR to turn on motor  co 10ms
ISR(TIMER0_COMP_vect)
{
  // roznica wartosci w tablicy random_values o 1 stanowi roznice w czasie o 500 ms
  // stad wartosci w random values grouped beda sie roznic o 500 jednakze to oznacza
  // ISR wywolywane jest co 10 ms stad 500 ms / 10 ms oznacza 50 przerwan w celu
  // wyznaczenia 0,5 s znacznika czasu
  //  static uint8 time_divider = 0;
  static uint8 rnd_val_gr_idx = 0;
  static uint16 rnd_val_cnt = 0;

  if (device_state == ST_WIBROWANIE)
  {

    if (rnd_val_cnt >= (random_values_grouped[rnd_val_gr_idx]/10))
    {
      rnd_val_cnt = 0;
      rnd_val_gr_idx++;
    }
    // indeksy parzyste oznaczaja wartosci w ktorych silnik jest uruchomiony
    // indeksy nieparzyste oznaczaja wartosci w ktorych silnik nie jest uruchomiony
    if (!(rnd_val_gr_idx % 2))
    {
      DEBUG_LED_ON;
      MOTOR_ON;
    }
    else
    {
      DEBUG_LED_OFF;
      MOTOR_OFF;
    }

    rnd_val_cnt++;

    if (rnd_val_gr_idx >= NUM_ACTIONS)
    {
      rnd_val_gr_idx = 0;
      device_state = ST_INTERAKCJA;
      #if DEBUG_STATE == _ON
      StrToSerial("No to powtarzaj, come on\n");
      #endif
    }
  }

}

//getting random variables
ISR(ADC_vect)
{
  static uint8_t rnd_idx = 0;

  if (device_state == ST_LOSOWANIE)
  {
    // jesli losowanie, losuj tak dlugo az index bedzie rowny 13, tyle bitow losujemy
    if (change_random)
    {
      random_lsb = ADC & 0x01;
      random_values[rnd_idx] = random_lsb;
      rnd_idx++;
    }

    // jesli wylosowano 13 bitow przejd do wibrowania, wylacz ADC
    if (rnd_idx >= NUM_RND)
    {
      // TODO zrob funkcje ktora wypelni rand val grouped wartosciami wylosowanymi
      rnd_idx = 0;
      change_random = 0;
      TurnADCOff;
      device_state = ST_WIBROWANIE;
      #if DEBUG_STATE == _ON
      StrToSerial("wylosowalem, wibruje\n");
      #endif
    }

  }
}

