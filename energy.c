/*
 * energy.c
 *
 *  Created on: 23 maj 2019
 *      Author: Dunajski
 */
#include <avr/interrupt.h>
#include "random.h"
#include "energy.h"

/*
 *******************************************************************************
 * Do ustawienia przetwornika zeby mierzyc napiecie zasilania.
 * Vref internal 2,56 V/ ADC1/ ISR EN/ Triggering manually/ fADC 125 kHz
 *******************************************************************************
 */
void SetAdcToMeasureSupplVoltage(void)
{
  ADMUX |= (1 << REFS0) | (1 << REFS1); // internal 2,56 V
  ADMUX |= (1 << MUX0); // ADC1 (PA1)
  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADIE); // Enable/ start / ISR EN
  ADCSRA |= (1 << ADPS1) | (1 << ADPS2); // prescaler 64 => 125 kHz
}


/*
 *******************************************************************************
 * Przerwanie od wybudzania MCU pzyciskiem.
 *******************************************************************************
 */
ISR(INT0_vect)
{
  // wylacz, zeby przeciwdzialac wielu wywolaniom, po obsludze przerwania
  // nastepuje uruchomienie wszystkich przerwan oprocz EXT0
  GICR &= ~(1 << INT0);
  // waking up...
  device_state = ST_WAIT_TO_WAKE_UP;
}

