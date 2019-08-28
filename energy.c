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
  // ADC ENABLE/start conversion/autotriger EN/interrupt execute EN/
  ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIE);
  // presk 64  f_adc=8MHz/64=125kHz
  ADCSRA |= (1 << ADPS1) | (1 << ADPS2);
}


/*
 *******************************************************************************
 * Przerwanie od wybudzania MCU pzyciskiem.
 *******************************************************************************
 */
ISR(INT0_vect)
{
  if (device_state == ST_POWER_DOWN)
  {
    // wylacz, zeby przeciwdzialac wielu wywolaniom, po obsludze przerwania
    // nastepuje uruchomienie wszystkich przerwan oprocz EXT0
    GICR &= ~(1 << INT0);
    // waking up...
    device_state = ST_WAIT_TO_WAKE_UP;
  }
}

