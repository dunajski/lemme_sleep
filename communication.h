/*
 * communication.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_
#include <avr/interrupt.h>


void PutUInt8ToSerial (uint8_t integer);
void PutToSerial(unsigned char data);
unsigned char GetFromSerial(unsigned char *p_dada);
void StrToSerial(char *msg);

#endif /* COMMUNICATION_H_ */
