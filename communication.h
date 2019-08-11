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

/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej o wielkosci 8b w postaci ASCII.
 * [in] uint8 integer - wartosc liczbowa do wyslania.
 *******************************************************************************
 */
void PutUInt8ToSerial(uint8 integer);

/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej o wielkosci 16b w postaci ASCII.
 * [in] uint16 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutUInt16ToSerial(uint16  value, uchar leading_zeros, uchar size);

/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej ze znakiem o wielkosci 16b w postaci ASCII.
 * [in] uint16 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutSInt16ToSerial(int16 value, uchar leading_zeros, uchar size);

/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej ze znakiem o wielkosci 32b w postaci ASCII.
 * [in] uint32 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutUInt32ToSerial(uint32 value, uchar leading_zeros, uchar size);

/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej ze znakiem o wielkosci 32b w postaci ASCII.
 * [in] sint32 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutSInt32ToSerial(int32 value, uchar leading_zeros, uchar size);

/*
 *******************************************************************************
 * Umozliwia umieszczenie w buforze nadawczym jednego znaku, po umieszczeniu
 * uruchamia przerwania od wysylania.
 * [in] uchar data - znak do wyslania.
 *******************************************************************************
 */
void PutToSerial(uchar data);

/*
 *******************************************************************************
 * Umozliwia umieszczanie danych z bufora pod wskazany adres.
 * [in] uchar * p_dada - adres do zapisu znakow z bufora odbiorczego.
 *******************************************************************************
 */
uchar GetFromSerial(uchar *p_dada);

/*
 *******************************************************************************
 * Umozliwia umieszczenie w buforze nadawczym lancucha znakowego.
 * [in] char * msg - lancuch znakow, do wyslania.
 *******************************************************************************
 */
void StrToSerial(char *msg);

#endif /* COMMUNICATION_H_ */
