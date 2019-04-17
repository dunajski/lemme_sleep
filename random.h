/*
 * random.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef RANDOM_H_
#define RANDOM_H_

#define NUM_RND 100
#define NUM_ACTIONS 5

#define TurnADCOn ADCSRA|=(1<<ADEN)|(1<<ADSC)
#define TurnADCOff ADCSRA&=~(1<<ADEN)

volatile unsigned char random_lsb;
volatile unsigned char change_random;
volatile unsigned char random_values[NUM_RND];
volatile unsigned char device_state;
// w tej tablicy musza sie znalezc wartosci interakcji dla poszczegolnych faz odbierania
// tj. dla puszczenia (przerwy) oraz przytrzymania klawisza, zapisywanie gdy ST_INTERAKCJA
// nalezy dodac debounce dla release i obsluge dodawania wartosci holda
unsigned char holdandreleasetime[NUM_ACTIONS];

uint8_t SaveHoldOrReleaseTime(unsigned char * hnr_array, uint8_t add_val);

#endif /* RANDOM_H_ */
