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
//#include "types.h"

int main(void)
{
  InitUart();
  sei();
  while (1)
    ;

  return 0;
}
