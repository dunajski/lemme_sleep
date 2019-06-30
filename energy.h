/*
 * energy.h
 *
 *  Created on: 23 maj 2019
 *      Author: Dunajski
 */

#ifndef ENERGY_H_
#define ENERGY_H_

extern volatile uint16 debounce_idle_delay;

/*
 *******************************************************************************
 * Do ustawienia przetwornika zeby mierzyc napiecie zasilania.
 * Vref internal 2,56 V/ ADC1/ ISR EN/ Triggering manually/ fADC 125 kHz
 *******************************************************************************
 */
void SetAdcToMeasureSupplVoltage(void);

#endif /* ENERGY_H_ */
