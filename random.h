/*
 * random.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef RANDOM_H_
#define RANDOM_H_

#include "types.h"
#include "peripherals.h"
#include "communication.h"

#define NUM_RND 100
#define NUM_ACTIONS 5

#define TurnADCOn ADCSRA|=(1<<ADEN)|(1<<ADSC)
#define TurnADCOff ADCSRA&=~(1<<ADEN)

volatile uchar random_lsb;
volatile uchar change_random;
volatile uchar random_values[NUM_RND];
volatile uchar device_state;

// w tej tablicy musza sie znalezc wartosci interakcji dla poszczegolnych faz odbierania
// tj. dla puszczenia (przerwy) oraz przytrzymania klawisza, zapisywanie gdy ST_INTERAKCJA
// nalezy dodac debounce dla release i obsluge dodawania wartosci holda
// 3 holdy 2 releasy
uint16 holdandreleasetime[NUM_ACTIONS];

#endif /* RANDOM_H_ */
