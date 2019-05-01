/*
 * random.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#include <avr/interrupt.h>
#include "random.h"

//ISR to turn on motor  co 10ms
ISR(TIMER0_COMP_vect)
{
//  static uint16 delay_cnt = 50;
//
//  if (device_state == ST_OCENA)
//  {
//    delay_cnt--;
//  }
//
//  if (delay_cnt == 0)
//  {
//    delay_cnt = 50;
//    device_state = ST_INTERAKCJA;
//  }
//  static uint16_t hnr_cnt = 0;
//  static uint8_t index_hnr = 0;
//
//  // rozne czasyd obu isr, jednakze sprawdzam dla debugu
//  if (device_state == ST_OCENA)
//  {
//    if (index_hnr == 0 || index_hnr == 2 || index_hnr == 4)
//      DEBUG_LED_ON;
//    else
//      DEBUG_LED_OFF;
//
//    if(hnr_cnt == 0)
//    {
//      hnr_cnt = holdandreleasetime[index_hnr];
//      index_hnr++;
//    }
//
//  }
//
//
//  if (hnr_cnt > 0)
//  {
//    hnr_cnt--;
//  }
//
//  if (index_hnr > 4)
//  {
//    index_hnr = 0;
//    device_state = ST_INTERAKCJA;
//  }
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

