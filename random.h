/*
 * random.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef RANDOM_H_
#define RANDOM_H_

#define TurnADCOn ADCSRA|=(1<<ADEN)|(1<<ADSC)
#define TurnADCOff ADCSRA&=~(1<<ADEN)

volatile unsigned char random_lsb;
volatile unsigned char change_random;

void PrintRandomBytes(void);
#endif /* RANDOM_H_ */
