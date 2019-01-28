/*
 * peripherals.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */
#include "peripherals.h"
#include <avr/io.h>

#define FIFO_LEN 128 //dlugosc kolejek FIFO

#define BAUDRATE 9600L//115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) // freq. divider

struct PortABits
{
  volatile unsigned char adc_pin :1;  // PA 0
  volatile unsigned char action_key :1;  // PA 1
  volatile unsigned char :3;  // PA 2-4
  volatile unsigned char motor :1;  // PA 5
  volatile unsigned char debug_led :1;  // PA 6
  volatile unsigned char state_led :1;  // PA 7
};

typedef struct PortABits TPortABits;

#define state_led_val ((TPortABits *)&PINA)->state_led
#define state_led_dir ((TPortABits *)&DDRA)->state_led
#define state_led_out ((TPortABits *)&PORTA)->state_led

#define debug_led_val ((TPortABits *)&PINA)->debug_led
#define debug_led_dir  ((TPortABits *)&DDRA)->debug_led
#define debug_led_out ((TPortABits *)&PORTA)->debug_led

#define motor_val ((TPortABits *)&PINA)->motor
#define motor_dir ((TPortABits *)&DDRA)->motor
#define motor_out ((TPortABits *)&PORTA)->motor

#define action_key_val ((TPortABits *)&PINA)->action_key
#define action_key_dir  ((TPortABits *)&DDRA)->action_key
#define action_key_pullup ((TPortABits *)&PORTA)->action_key

#define adc_pin_val ((TPortABits *)&PINA)->adc_pin
#define adc_pin_dir ((TPortABits *)&DDRA)->adc_pin
#define adc_pin_pullup ((TPortABits *)&PORTA)->adc_pin

void InitUart(void)
{
  UBRRH = (BAUD_REG >> 8);
  UBRRL = BAUD_REG;
  //Format ramki danych: 8data,1stopbit
  UCSRC |= ((1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1));
  UCSRB |= ((1 << RXEN) | (1 << TXEN));  // RX/TX enable
  UCSRB |= (1 << RXCIE);  //RX ISR enable
}
void InitTimer0(void)
{
  // ISR execute period 10 ms / interrupt /  CTC   // f CTC = fio/(2*presc*(1+OCR) -> t = 10 ms 1/t = fCTC -> 1/10ms -> 100 Hz
  // OCR ~= 77 5ms , OCR ~=155 10ms // presc 256 8 000 000 / 256 = 31250
  TCCR0 |= (1 << WGM01) | (1 << CS02);  // prescaler 256  | CTC mode
  TIMSK |= (1 << OCIE0);  //ctc timer0 isr enable
  OCR0 = 155;
}
void InitTimer2(void)
{
  OCR2 = 199;  // F_CPU 8Mhz/16MHz przerwanie co 0,4ms/0,2ms
  TCCR2 |= (1 << WGM21 | 1 << CS21);  //CTC pres 8
  TIMSK |= (1 << OCIE2);  //ctc timer2 isr enable
}
void InitAdc(void)
{
  ADMUX |= (1 << REFS0);  //drfting pin
  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
  // ADC ENABLE/start conversion/autotriger EN/interrupt execute EN/ presk 64  f_adc=8MHz/64=125kHz
}

void InitIO(void)
{
  state_led_dir = 1;
  debug_led_dir = 1;
  motor_dir  = 1;
  action_key_dir = 0;
  adc_pin_dir = 0;

  action_key_pullup = 1;
  adc_pin_pullup = 0;
}




