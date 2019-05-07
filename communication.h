/*
 * communication.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <avr/interrupt.h>
#include "types.h"
#include "random.h"

extern unsigned char * p_dada;

void PutUInt8ToSerial(uint8 integer);
void PutUInt16ToSerial(uint16  value, uchar leading_zeros, uchar size);
void PutSInt32ToSerial(int32 value, uchar leading_zeros, uchar size);
void PutToSerial(uchar data);
uchar GetFromSerial(uchar *p_dada);
void StrToSerial(char *msg);

#endif /* COMMUNICATION_H_ */
