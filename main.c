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

unsigned char * p_dada;

int main(void)
{
  InitUart();
  InitAdc();
  InitTimer0();
  InitTimer2();
  InitIO();
  uint8_t i = 0;
  sei();
  while (1)
  {
    if (GetFromSerial(p_dada))
    {
      PutUInt8ToSerial(i++);
      if (i > UINT8_MAX)
        i = 0;
    }
  }

  return 0;
}
