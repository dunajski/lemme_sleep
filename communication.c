/*
 * communication.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */
#include <avr/io.h>
#include <string.h>
#include "communication.h"

static uint8 ConvertUInt16ToAscii(uint16 value, uchar * buffer, uchar leading_zeros, uchar size);
static uint8 ConverUInt32ToAscii(uint32 value, uchar * buffer, uchar leading_zeros, uchar size);

/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej o wielkosci 8b w postaci ASCII.
 * [in] uint8 integer - wartosc liczbowa do wyslania.
 *******************************************************************************
 */
void PutUInt8ToSerial(uint8 integer)
{
  if (integer >= 100)
    PutToSerial((((integer % 1000) - (integer % 100)) / 100 + '0'));  // setki
  if (integer >= 10)
    PutToSerial((((integer % 100) - (integer % 10)) / 10) + '0');  // dziesiatki

  PutToSerial(integer % 10 + '0');
}

#define _UINT16_MAX_ASCII_DIGITS 5 // maks 65 535
/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej o wielkosci 16b w postaci ASCII.
 * [in] uint16 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutUInt16ToSerial(uint16 value, uchar leading_zeros, uchar size)
{
  uint8 how_many_digits = 0;
  uint8 ascii[_UINT16_MAX_ASCII_DIGITS];

  how_many_digits = ConvertUInt16ToAscii(value, ascii, leading_zeros, size);

  for (int i = 0; i < how_many_digits; i++)
    PutToSerial(ascii[i]);
}

#define _SINT16_MAX_ASCII_DIGITS 6
/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej ze znakiem o wielkosci 16b w postaci ASCII.
 * [in] uint16 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutSInt16ToSerial(int16 value, uchar leading_zeros, uchar size)
{
  uint8 how_many_digits = 0;
  uint8 ascii[_SINT16_MAX_ASCII_DIGITS];
  uint8 i = 0;

  if (0 > value)
  {
    value = value * (-1);
    PutToSerial('-');
    i++;
  }

  how_many_digits = ConvertUInt16ToAscii((uint16)value, ascii, leading_zeros, size);

  for (; i < how_many_digits; i++)
    PutToSerial(ascii[i]);
}

#define _UINT32_MAX_ASCII_DIGITS 10
/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej ze znakiem o wielkosci 32b w postaci ASCII.
 * [in] uint32 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutUInt32ToSerial(uint32 value, uchar leading_zeros, uchar size)
{
  uint8 how_many_digits = 0;
  uint8 ascii[_UINT32_MAX_ASCII_DIGITS];
  uint8 i = 0;

  how_many_digits = ConverUInt32ToAscii(value, ascii, leading_zeros, size);

  for (i = 0; i < how_many_digits; i++)
    PutToSerial(ascii[i]);
}

// maks moze byc 10, ale dodaje w buforze jedno miejsce na '-'
#define _SINT32_MAX_ASCII_DIGITS 11
/*
 *******************************************************************************
 * Umozliwia wyslanie zmiennej ze znakiem o wielkosci 32b w postaci ASCII.
 * [in] sint32 value - wartosc liczbowa do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
void PutSInt32ToSerial(int32 value, uchar leading_zeros, uchar size)
{
  uint8 how_many_digits = 0;
  uint8 ascii[_SINT32_MAX_ASCII_DIGITS];
  uint8 i = 0;

  if (0 > value)
  {
    value = value * (-1);
    PutToSerial('-');
    i++;
  }

  how_many_digits = ConverUInt32ToAscii((uint32)value, ascii, leading_zeros, size);

  for (; i < how_many_digits; i++)
    PutToSerial(ascii[i]);
}

/*
 *******************************************************************************
 * Umozliwia konwersje zmiennej o wielkosci 16b do postaci ASCII.
 * [in] uint16 value - wartosc liczbowa do wyslania,
 * [in] uchar * buffer - bufor na znaki do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
static uint8 ConvertUInt16ToAscii(uint16 value, uchar * buffer, uchar leading_zeros, uchar size)
{
  uint8 length = 0;
  uint8 ascii[_UINT16_MAX_ASCII_DIGITS];

  do
  {
    if (length >= size)
      break;

    ascii[_UINT16_MAX_ASCII_DIGITS - 1 - length] = (value % 10) + '0';
    value /= 10;
    length++;
  } while (value > 0);

  if (leading_zeros)
  {
    memset(buffer, '0', size - length);
    buffer += size - length;
  }

  memcpy(buffer, &ascii[_UINT16_MAX_ASCII_DIGITS - length], length);

  return leading_zeros ? size : length;
}

/*
 *******************************************************************************
 * Umozliwia konwersje zmiennej o wielkosci 32b do postaci ASCII.
 * [in] uint32 value - wartosc liczbowa do wyslania,
 * [in] uchar * buffer - bufor na znaki do wyslania,
 * [in] uchar leading_zeros - umozliwia wyslanie zer poprzedzajacych,
 * [in] uchar size - ilosc znakow do wyslania,
 * [out] uint8 - ilosc wyslanych znakow.
 *******************************************************************************
 */
static uint8 ConverUInt32ToAscii(uint32 value, uchar * buffer, uchar leading_zeros, uchar size)
{
  uint8 length = 0;
  uint8 ascii[_SINT32_MAX_ASCII_DIGITS]; // 1 for '-' if needed

  do
  {
    if (length >= size)
      break;

      ascii[_SINT32_MAX_ASCII_DIGITS - 1 - length] = (value % 10) + '0';

    value /= 10;
    length++;
  } while (value > 0);

  if (leading_zeros)
  {
    memset(buffer, '0', size - length);
    buffer += size - length;
  }

  memcpy(buffer, &ascii[_SINT32_MAX_ASCII_DIGITS - length], length);

  return leading_zeros ? size : length;
}

#define FIFO_LEN 512 // dlugosc kolejek FIFO

struct
{
  uchar wi;  //indeks odczytu
  uchar ri;  //indeks zapisu
  char buff[FIFO_LEN];
}
InputFifo = {0, 0},
OutputFifo = {0, 0};

/*
 *******************************************************************************
 * Przerwanie umieszcza odebrane dane w buforze odbiorczym.
 *******************************************************************************
 */
ISR(USART_RXC_vect) /*VECTOR(11), USART, RxComplete*/
{
  InputFifo.buff[InputFifo.wi++] = UDR;  //umieszczenie danej w kolejce
  if (InputFifo.wi == FIFO_LEN)
    InputFifo.wi = 0;
}

/*
 *******************************************************************************
 * Umozliwia umieszczanie danych z bufora pod wskazany adres.
 * [in] uchar * p_dada - adres do zapisu znakow z bufora odbiorczego.
 *******************************************************************************
 */
uchar GetFromSerial(uchar * p_dada)
{
  if (InputFifo.ri == InputFifo.wi)
    return 0;
  else
  {
    *p_dada = InputFifo.buff[InputFifo.ri++];
    if (InputFifo.ri == FIFO_LEN)
      InputFifo.ri = 0;
    return 1;
  }
}

/*
 *******************************************************************************
 * Przerwanie obslugujace nadawanie przy uzyciu USART. Wysyla znaki tak dlugo
 * jak ma zapelniony bufor. Gdy bufor sie zwolni oczekuje na wlaczenie
 * przerwania po wrzuceniu danych do wyslania do bufora.
 *******************************************************************************
 */
ISR(USART_UDRE_vect) /*VECTOR(12), USART Data Register Empty*/
{
  if (OutputFifo.wi == OutputFifo.ri)
  {
    UCSRB &= ~(1 << UDRIE);  //bufor FIFO pusty - wylaczenie przerwania
  }
  else
  {
    UDR = OutputFifo.buff[OutputFifo.ri++];  //dana z bufora do rejestru UDR
    if (OutputFifo.ri == FIFO_LEN)
      OutputFifo.ri = 0;
  }
}

/*
 *******************************************************************************
 * Umozliwia umieszczenie w buforze nadawczym jednego znaku, po umieszczeniu
 * uruchamia przerwania od wysylania.
 * [in] uchar data - znak do wyslania.
 *******************************************************************************
 */
void PutToSerial(uchar data)
{
  OutputFifo.buff[OutputFifo.wi++] = data;

  if (OutputFifo.wi == FIFO_LEN)
    OutputFifo.wi = 0;
  UCSRB |= (1 << UDRIE);  // wlaczenie przerwan
}

/*
 *******************************************************************************
 * Umozliwia umieszczenie w buforze nadawczym lancucha znakowego.
 * [in] char * msg - lancuch znakow, do wyslania.
 *******************************************************************************
 */
void StrToSerial(char * msg)
{
  while (*msg != 0)
    PutToSerial(*msg++);
}

