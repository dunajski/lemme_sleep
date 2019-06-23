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

#define NUM_RND     13
#define NUM_ACTIONS  5

#define TurnADCOn   (ADCSRA|=(1<<ADEN)|(1<<ADSC))
#define TurnADCOff  (ADCSRA&=~(1<<ADEN))
#define StartADConversion (ADCSRA|=(1<<ADSC))

volatile uchar random_lsb;
volatile uchar change_random;
volatile uchar random_values[NUM_RND];
volatile uchar device_state;

// w tej tablicy beda sie znajdowac wartosci czasow wibrowania sekwencji
// pogrupowane ze wzgledu na to ze zostana zsumowane z tablicy random values
// dla kazdego czasu WIBROWANIA przedzial od 0,5 do 3,5 s oraz
// dla kazdego czasu PRZERWY przedzial od 0,5 do 1,5 s
// wartosci moglybybyc jako uchar a nastepnie mnozone w przerwaniu, ale takie
// poszerzenie rozdzielczosci umozliwi prostsze dzialanie na tych liczbach
// w trakcie oceny aktywnosci uzytkownika
extern volatile uint16 random_values_grouped[NUM_ACTIONS];

// w tej tablicy musza sie znalezc wartosci interakcji dla poszczegolnych faz odbierania
// tj. dla puszczenia (przerwy) oraz przytrzymania klawisza, zapisywanie gdy ST_INTERAKCJA
// nalezy dodac debounce dla release i obsluge dodawania wartosci holda
// 3 holdy 2 releasy
volatile uint16 holdandreleasetime[NUM_ACTIONS];

// dla obu tablic tj. holdandreleasetime i random values grouped
// indeksy 0,2,4 to czasy WIBROWANIA silnikiem/TRZYMANIA przycisku
// indeksy 1,3 to czasy PRZERW w wibrowaniu silnikiem/PUSZCZENIA przycisku

#endif /* RANDOM_H_ */
