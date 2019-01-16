/*
 * main.c
 *      Author: Dunajski
 *      thesis
 *
 */
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "types.h"

#define BAUDRATE 9600L//115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) //dzielnika cz. UBRR

void InitUart(void)  //Inicjowanie portu UART
{
  UBRRH = (BAUD_REG >> 8);  //dzielnk czestotliwosci transmisji
  UBRRL = BAUD_REG;
  //Format ramki danych: 8data,1stopbit
  UCSRC |= ((1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1));
  UCSRB |= ((1 << RXEN) | (1 << TXEN));  //zezwolenie na odbior i nadawanie
  UCSRB |= (1 << RXCIE);  //zezwolenie na przerwanie odb.
}

int main(void)
{
  InitUart();
  sei();
  while (1)
    ;

  return 0;
}
