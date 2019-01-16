/*
 * communication.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */
//TODO add template, for .c , .h
//TODO check static needed
#include <avr/io.h>
#include "communication.h"
#include "types.h"

#define FIFO_L 128 //dlugosc kolejek FIFO

void PutIntToSerial (uint8_t integer) // zamiana INT'a na chara
{
  if(integer >=100)
  {
    PutToSerial((((integer%1000)-(integer%100))/100 + 0x30)); // setki
  }
  if(integer >=10)
  {
    PutToSerial((((integer%100)-(integer%10))/10) + 0x30); // dziesiatki
  }
  PutToSerial(integer%10 + 0x30);
}

//cz�stotliwo�� zegara mikrokotrolera
#define BAUDRATE 9600L//115200L //pr�dko�� transmisji w bodach
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) //dzielnika cz. UBRR
//---------------------------------------------------------------
void InitUart(void) //Inicjowanie portu UART
{
  UBRRH=(BAUD_REG>>8); //dzielnk czestotliwosci transmisji
  UBRRL=BAUD_REG;
  //Format ramki danych: 8data,1stopbit
  UCSRC |=((1<<URSEL)| (1<<UCSZ0)|(1<<UCSZ1));
  UCSRB |=((1<<RXEN)| (1<<TXEN)); //zezwolenie na odbior i nadawanie
  UCSRB |=(1<<RXCIE); //zezwolenie na przerwanie odb.
}


struct {
  unsigned char wi; //indeks odczytu
  unsigned char ri; //indeks zapisu
  char buff[FIFO_L];
}
  InputFifo ={0,0},
  OutputFifo={0,0};

ISR(USART_RXC_vect) /*VECTOR(11), USART, RxComplete*/
{
  InputFifo.buff[InputFifo.wi++]=UDR; //umieszczenie danej w kolejce
  if (InputFifo.wi == FIFO_L) InputFifo.wi = 0;
}
//----------------------------------------------------
// zwraca: 0 -gdy bufor odbiornika pusty
// 1 -gdy pobrany znak umieszczony w '*p_dada'
unsigned char GetFromSerial(unsigned char *p_dada)
{
  if (InputFifo.ri == InputFifo.wi) return 0;

  else {
    *p_dada = InputFifo.buff[InputFifo.ri++];
    if (InputFifo.ri == FIFO_L) InputFifo.ri=0;
    return 1;
  }
}

ISR(USART_UDRE_vect) /*VECTOR(12), USART Data Register Empty*/
{
  if (OutputFifo.wi == OutputFifo.ri)
  {
    UCSRB &= ~(1<<UDRIE); //bufor FIFO pusty - wylaczenie przerwania
  }
  else {
    UDR = OutputFifo.buff[OutputFifo.ri++]; //dana z bufora do rejestru UDR
    if (OutputFifo.ri == FIFO_L) OutputFifo.ri = 0;
  }
}
//----------------------------------------------------------------
void PutToSerial(unsigned char data)
{
  OutputFifo.buff[OutputFifo.wi++] = data;

  if (OutputFifo.wi == FIFO_L ) OutputFifo.wi = 0;
  UCSRB |=(1<<UDRIE); // wlaczenie przerwan
}

void StrToSerial(char *msg)
{
  while(*msg!=0) PutToSerial(*msg++);
}


