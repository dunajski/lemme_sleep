/*
 * communication.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */
#include <avr/io.h>
#include "types.h"
#include "communication.h"
#include "random.h"

#define FIFO_LEN 128 //dlugosc kolejek FIFO

#define BAUDRATE 9600L//115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) //dzielnika cz. UBRR

void PutUInt8ToSerial(uint8_t integer)
{
  if (integer >= 100)
    PutToSerial((((integer % 1000) - (integer % 100)) / 100 + '0'));  // setki
  if (integer >= 10)
    PutToSerial((((integer % 100) - (integer % 10)) / 10) + '0');  // dziesiatki

  PutToSerial(integer % 10 + '0');
}

struct
{
  unsigned char wi;  //indeks odczytu
  unsigned char ri;  //indeks zapisu
  char buff[FIFO_LEN];
}
InputFifo = {0, 0},
OutputFifo = {0, 0};

ISR(USART_RXC_vect) /*VECTOR(11), USART, RxComplete*/
{
  InputFifo.buff[InputFifo.wi++] = UDR;  //umieszczenie danej w kolejce
  if (InputFifo.wi == FIFO_LEN)
    InputFifo.wi = 0;
}

// zwraca: 0 -gdy bufor odbiornika pusty
// 1 -gdy pobrany znak umieszczony w '*p_dada'
unsigned char GetFromSerial(unsigned char * p_dada)
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

void PutToSerial(unsigned char data)
{
  OutputFifo.buff[OutputFifo.wi++] = data;

  if (OutputFifo.wi == FIFO_LEN)
    OutputFifo.wi = 0;
  UCSRB |= (1 << UDRIE);  // wlaczenie przerwan
}

void StrToSerial(char *msg)
{
  while (*msg != 0)
    PutToSerial(*msg++);
}

