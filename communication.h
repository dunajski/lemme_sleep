/*
 * communication.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <avr/interrupt.h>

extern unsigned char * p_dada;

void PutUInt8ToSerial(uint8_t integer);
void PutUint16ToSerial(uint16_t  value);
void PutToSerial(unsigned char data);
unsigned char GetFromSerial(unsigned char *p_dada);
void StrToSerial(char *msg);
uint8_t ConversionUInt16ToAscii(uint16_t value, unsigned char * buffer);

#endif /* COMMUNICATION_H_ */
