/*
 * peripherals.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */
//copy paste check it
#include "peripherals.h"
#include <avr/io.h>

#define FIFO_L 128 //dlugosc kolejek FIFO

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
void InitTimer0(void)
{
  // Interwal czasu 10 ms / interrupt /  CTC   // f CTC = fio/(2*presc*(1+OCR) -> t = 10 ms 1/t = fCTC -> 1/10ms -> 100 Hz
  // OCR ~= 77 5ms , OCR ~=155 10ms // presc 256 8 000 000 / 256 = 31250
  TCCR0 |= (1 << WGM01) | (1 << CS02);  // // preskaler 256  | CTC
  TIMSK |= (1 << OCIE0);  // flaga przerwania | wykonanie przerwana
  OCR0 = 155;
}
void InitTimer2(void)
{
  OCR2 = 199;  // F_CPU 8Mhz/16MHz przerwanie co 0,4ms/0,2ms
  TCCR2 |= (1 << WGM21 | 1 << CS21);  //CTC pres 8
  TIMSK |= (1 << OCIE2);  //Odblokowanie przerwan
}
void InitAdc(void)
{
  ADMUX |= (1 << REFS0);  // avcc[niepodlaczony kondek na avcc a gnd-celowo]
  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
  // wl. ADC/start konwersji/autotriger EN/interrupt execute EN/ presk 64  f_adc=8MHz/64=125kHz
}

void InitIO (void)
{
  //TODO input output key etc.
}
