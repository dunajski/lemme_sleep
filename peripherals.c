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

#define ISR_DEBOUNCE_CNT 200

#define BUTTON_NOT_PRESSED 0  // przycisk niewcisniety podczas stanu INTERAKCJA
#define BUTTON_PRESSED 1      // przycisk wcisniety podczas stanu INTERAKCJA

#define FIFO_LEN 128 //dlugosc kolejek FIFO

#define BAUDRATE 9600L//115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) // freq. divider

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
//  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
  ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
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

  device_state = ST_LOSOWANIE; // na razie rozpocznij od razu od losowania
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
  static unsigned char * hnr_time_ptr = holdandreleasetime;
  static uint16_t button_state_time = 0; // powinnien 16 bitowy wystarczyc 0,4 ms x 0xFFFF = ~26 s
  static uint8_t substate_interakcja = 0;
//  static uint16_t draw_random_cnt = 0;

  // ISR co 0,4ms co tyle, losujemy random lsb
  // zeby nie "zapchac" kanalu trasnmisyjnego
  // stad volatile od wysylania bedzie zmieniany tutaj
  // jeszcze od state bedzie zmieniane, jednakze na razie jest tak

  change_random_cnt++;

  if (change_random_cnt >= 100) // co 200ms losowanie kolejnej liczby
  {
    change_random = 1;

    TOGGLE_BIT(PORTA,PA6);
//    draw_random_cnt++;
    change_random_cnt = 0;
  }

//  if (draw_random_cnt >= 10000)
//  {
//    TurnADCOff;
//  }
//

  // obsluga klawisza, jedynego pewnie wiec nalezy uzaleznic od
  // stanu urzadzenia (TDeviceStates)
  if (keycnt == 0)
  {
    switch (keylev)
    {
      case 0:  // waiting for press
        if (!ACTION_KEY_VAL)
        {
          keyr = ACTION_KEY_VAL;
          keylev = 1;
          keycnt = ISR_DEBOUNCE_CNT;
        }
      break;
      case 1: // pressed,debounced
        if (ACTION_KEY_VAL == keyr)
        {
          if (device_state != ST_INTERAKCJA)
            keylev = 2;
          else


          switch (device_state)
          {
            case ST_IDLE:
              // nalezaloby jaks rozbudzic, finalnie moze sie okazac, ze lepiej
              // zeby klawiszem ktorym sterujemy byl inny niz ten na PortA,a tam gdzie EXT0/1
            break;
            case ST_WIBROWANIE:
              // sterowanie silnikiem ze wzgledu na wylosowane wartosci
              // obsluga klawisza do dyskusji bo uzytkownik powininen
              // starac sie zapamietac losowa wibracje
            break;
            case ST_INTERAKCJA:
              // obsluga, zapisu interakcji najitotniejsze czyli mierzenie odpowiedzi uzytkownika
              // nastepnym stanem tj. po uplynieciu maksymalnego czasu oczekiwania na odopowiedz
              // powinien byc stan podjecia decyzji o tym jak ma wygladac kolejne nadawanie
              // po debounce dodaj wartosc debounce az do puszczenia przycisku
              // debounce po puszczeniu

              // jesli wciaz wcisniety i interakcja to dodawaj button state, tak
              // dlugo az uzytkownik pusci dzwignie/przycisk
              if (ACTION_KEY_VAL == 1 )
              {
                button_state_time++;
//                hnr_time_ptr += SaveButtonStateTime(hnr_time_ptr, 100);
              }
              // jesli uzytkownik puscil przycisk to przestan naliczac button_state_time i zacznij
              // debounce dla puszczenia przycisku
            break;
            case ST_OCENA:
            case ST_LOSOWANIE:
            default:
              // do nothing
            break;
          }
        // StrToSerial("Probki:\n");
        // draw_random_cnt = 0;
        // TurnADCOn;
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

