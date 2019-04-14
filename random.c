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

//void PrintRandomBytes(void)
//{
//  for (int i = 0; i < 10/*_NumItems(random_bytes)*/; i++)
//    PutToSerial(random_bytes[i]);
//}

//ISR to turn on motor
ISR(TIMER0_COMP_vect)
{

}

//getting random variables
ISR(ADC_vect)
{
//  static uint16_t cnt = 0;

  if(change_random)
  {
    random_lsb = ADC & 0x01;
    PutUInt8ToSerial(random_lsb);
    StrToSerial("\n");
    change_random = 0;
  }

//  cnt++;
//
//  if (cnt >= 1000)
//  {
//    TurnADCOff;
//    cnt = 0;
//  }
}
