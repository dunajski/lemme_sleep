/*
 * random.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#include <avr/interrupt.h>
#include "random.h"
#include "types.h"
#include "peripherals.h"
#include "communication.h"

//ISR to turn on motor
ISR(TIMER0_COMP_vect)
{

}

//getting random variables
ISR(ADC_vect)
{
  static uint8_t rnd_idx = 0;

  if(change_random)
  {
    random_lsb = ADC & 0x01;
//    PutUInt8ToSerial(random_lsb);
//    StrToSerial("\n");
    random_values[rnd_idx] = random_lsb;
    PutUInt8ToSerial(rnd_idx + 1);
    StrToSerial(":");
    PutUInt8ToSerial(random_values[rnd_idx]);
    StrToSerial("\n");
    rnd_idx++;
    change_random = 0;
  }

  if (rnd_idx >= NUM_RND)
  {
    rnd_idx = 0;
    TurnADCOff;
  }
}

uint8_t SaveButtonStateTime(uint16_t * hnr_array, uint16_t time)
{
  *hnr_array += time;
  // zwraca 1 zeby przesunac wskaznik, potem mozna dac ** i przesuwac wewnatrz
  // lub po prostu przesuwac za kazdym wywolaniem w kodzie, na razie tak
  return 1;
}
