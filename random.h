/*
 * random.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef RANDOM_H_
#define RANDOM_H_

#define NUM_RND 100

#define TurnADCOn ADCSRA|=(1<<ADEN)|(1<<ADSC)
#define TurnADCOff ADCSRA&=~(1<<ADEN)

volatile unsigned char random_lsb;
volatile unsigned char change_random;
volatile unsigned char random_values[NUM_RND];

#endif /* RANDOM_H_ */
