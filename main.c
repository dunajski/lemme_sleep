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
  sei();
  while (1)
  {
  }

  return 0;
}
