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

unsigned char random_bytes[10];

void PrintRandomBytes(void)
{
  for (int i = 0; i < 10/*_NumItems(random_bytes)*/; i++)
    PutToSerial(random_bytes[i]);
}

//ISR to turn on motor
ISR(TIMER0_COMP_vect)
{

}

//getting random variables
ISR(ADC_vect)
{
  static uint8_t rnd_idx = 0;

//  random_bytes[rnd_idx] = ADC;
  PutToSerial((unsigned char)ADC);
  rnd_idx++;

  if (rnd_idx >= _NumItems(random_bytes))
    rnd_idx = 0;

  TurnADCOff;
}
