/*
 * peripherals.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#include <avr/interrupt.h>
#include <avr/iom32.h>
#include <stdint.h>

#include "communication.h"
#include "peripherals.h"
#include "random.h"
#include "types.h"

#define FIFO_LEN 128 //dlugosc kolejek FIFO

#define BAUDRATE 9600L//115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) // freq. divider

typedef enum DeviceStates
{
  POWER_SAFE,
  LOSOWANIE,
  NADAWANIE,
  INTERAKCJA,
} TDeviceState;

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
  ADMUX |= (1 << REFS0);  //drfting pin lepiej na AREFie bez niczego
  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
  // ADC ENABLE/start conversion/autotriger EN/interrupt execute EN/ presk 64  f_adc=8MHz/64=125kHz
}

void InitIO(void)
{
  STATE_LED_DIR = 1;
  DEBUG_LED_DIR = 1;

  MOTOR_DIR = 1;

  ACTION_KEY_PULLUP = 1;
  ACTION_KEY_DIR = 0;

  ADC_PIN_DIR = 0; // wejscie
  ADC_PIN_PULLUP = 0; // bez pullupu, niech dryfuje
}

// isr to debounce key and measure feedback
//ISR 0,4ms
// keycnt 200 then 0,4 * 200 = 80ms
ISR(TIMER2_COMP_vect)
{
  static uint16_t keycnt = 0;
  static uint8_t keylev = 0;
  static uint8_t keyr = 0;
  static uint16_t change_random_cnt = 0;
  static uint16_t draw_random_cnt = 0;

  // ISR co 0,4ms co tyle, losujemy random lsb
  // zeby nie "zapchac" kanalu trasnmisyjnego
  // stad volatile od wysylania bedzie zmieniany tutaj
  // jeszcze od state bedzie zmieniane, jednakze na razie jest tak

  change_random_cnt++;

  if (change_random_cnt >= 5000) // co 200ms losowanie kolejnej liczby
  {
    change_random = 1;

    TOGGLE_BIT(PORTA,PA6);
    draw_random_cnt++;
    change_random_cnt = 0;
  }

  if (draw_random_cnt >= 15)
  {
    TurnADCOff;
  }


  if (keycnt == 0)
  {
    switch (keylev)
    {
      case 0:  // waiting for press
        if (!ACTION_KEY_VAL)
        {
          keyr = ACTION_KEY_VAL;
          keylev = 1;
          keycnt = 200;
        }
      break;
      case 1: // pressed,debounced
        if (ACTION_KEY_VAL == keyr)
        {
          keylev = 2;
          StrToSerial("NEXT:\n");
          draw_random_cnt = 0;
          TurnADCOn;
        }
      break;
      case 2:
        if (ACTION_KEY_VAL == 1)
          keylev = 0;
      break;
    }
  }

  if (keycnt > 0)
    keycnt--;
}

