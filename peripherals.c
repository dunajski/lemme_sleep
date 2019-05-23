/*
 * peripherals.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#include <avr/interrupt.h>
#include <avr/iom32.h>
#include <stdint.h>
#include "peripherals.h"

#define ISR_DEBOUNCE_CNT 200
#define _BREAK_TIME_S(x) (10000UL*x)

#define TRUE (1)
#define FALSE (0)

volatile uint16 * hnr_time_ptr = holdandreleasetime;
volatile uchar how_many_times_sent = 0;

//static uint8 EstimateActivity(uint16 hnr_time[], uchar rnd_values[], uint8 current_activity);

// stany przycisku podczas odliczania czasu wcisnieta i puszczenia dla stanu ST_INTERAKCJA
typedef enum KeySubState
{
  INITIAL_KEY_STATE = 0,  // stan przed wcisnieciem pierwszym
  BUTTON_NOT_PRESSED,     // stan puszczenia, ale musi uprzednio wystapic choc jedno wcisniecie
  BUTTON_PRESSED,         // stan do rozpoczecia odlcizania wcisniecia, oraz ogolnego rozpoczenia mierzenia
  BUTTON_IS_HOLDED,       // stan do naliczania czasu po debounce po wcisnieciu przycisku
  BUTTON_IS_RELEASED      // stan do naliczania czasu po debounce po zwolnieniu przycisku

} TKeySubstates;

#define FIFO_LEN 128  //dlugosc kolejek FIFO

#define BAUDRATE 9600UL  //115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) // freq. divider

void InitUart(void)
{
  UBRRH = (BAUD_REG >> 8);
  UBRRL = BAUD_REG;
  //Format ramki danych: 8data,1stopbit
  UCSRC |= ((1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1));
  UCSRB |= ((1 << RXEN) | (1 << TXEN));  // RX/TX enable
  UCSRB |= (1 << RXCIE);                 //RX ISR enable
}

void InitTimer0(void)
{
  // ISR execute period 10 ms / interrupt /  CTC   // f CTC = fio/(presc*(1+OCR)
  // -> t = 10 ms 1/t = fCTC -> 1/10ms -> 100 Hz
  TCCR0 |= (1 << WGM01) | (1 << CS02) | (1 << CS00);  // prescaler 1024  | CTC mode
  TIMSK |= (1 << OCIE0);                              //ctc timer0 isr enable
  OCR0 = 77;
}

void InitTimer2(void)
{
  OCR2 = 199;  // F_CPU 8MHz przerwanie co 0,2ms f CTC = fio/(presc*(1+OCR))
  // -> f CTC = 8 000 000 / (8 * 200) = 5000 => 1/5000 = 0,2 ms
  TCCR2 |= (1 << WGM21 | 1 << CS21);  //CTC | presc 8
  TIMSK |= (1 << OCIE2);              //CTC timer2 isr enable
}

void InitAdc(void)
{
  ADMUX |= (1 << REFS0);  //drifting pin lepiej na AREFie bez niczego
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
  ACTION_KEY_DIR    = 0;

  ADC_PIN_DIR    = 0; // wejscie
  ADC_PIN_PULLUP = 0; // bez pullupu, niech dryfuje

  device_state = ST_WIBROWANIE;

// to co ma sie zrobic przed petla glowna
#if DEBUG_STATE == _ON
  PutSInt32ToSerial(-10, TRUE, 15);
  StrToSerial("\n");
  PutSInt32ToSerial(123112L, TRUE, 15);
  StrToSerial("\n");
  PutUInt16ToSerial(random_values_grouped[0], FALSE, 8);
  StrToSerial("\n");
  PutUInt16ToSerial(random_values_grouped[1], FALSE, 8);
  StrToSerial("\n");
  PutUInt16ToSerial(random_values_grouped[2], FALSE, 8);
  StrToSerial("\n");
  PutUInt16ToSerial(random_values_grouped[3], FALSE, 8);
  StrToSerial("\n");
  PutUInt16ToSerial(random_values_grouped[4], FALSE, 8);
#endif
}

// isr to debounce key and measure feedback
//ISR 0,2ms
// keycnt 200 then 0,2 * 200 = 40ms
ISR(TIMER2_COMP_vect)
{
  static uint8  keyr = 0;
  static uint8  index = 0;
  static uint8  keylev = 0;
  static uint8  saved_states = 0;
  static uint8  key_state_interakcja = INITIAL_KEY_STATE;
  static uint16 keycnt = 0;
  static uint16 keycnt2 = 0;
  static uint16 button_state_time = 0; // powinnien 16 bitowy wystarczyc 0,4 ms x 0xFFFF = ~26 s
//  static uint16 change_random_cnt = 0;
//  static uint8 activity_rate;


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

          switch (device_state)
          {
            case ST_IDLE:
              // w przerwaniu od INT1, dodaj jakiegos statica 3 razy
            break;
            case ST_WIBROWANIE:
              // w trakcie wibrowania nie obchodzi nas czy ktos klika
            break;
            case ST_INTERAKCJA:
              // odbieranie sekwencji od uzytkownika
            break;
            case ST_OCENA:
            case ST_LOSOWANIE:
            default:
              // do nothing
            break;
          }
        }
        // jesli stan sie rozni
        else
          keylev = 0;
      break;
      case 2:
        if (ACTION_KEY_VAL == 1)
          keylev = 0;
      break;
    }
  }

  if (keycnt > 0)
    keycnt--;

  if (device_state == ST_INTERAKCJA)
  {

    if (!keycnt2 && (saved_states < NUM_ACTIONS))
    {

      // aby rozpoczac mierzenie hold and release musi byc stan init, zeby mozna bylo rozpoczac
      // mierzenie od wcisniecia, a wiec oczekuje na wcisniecie przycisku
      if ((key_state_interakcja == INITIAL_KEY_STATE) && !(ACTION_KEY_VAL))
      {
        // debouncing wcisnieto przycisk
        keycnt2 = ISR_DEBOUNCE_CNT;
        key_state_interakcja = BUTTON_PRESSED;
      }

      // sprawdzamy czy po debounce przycisk wciaz jest przycisniety
      if (key_state_interakcja == BUTTON_PRESSED)
      {
        if (!(ACTION_KEY_VAL))
        {
          button_state_time = ISR_DEBOUNCE_CNT;  // dodanie czasu debounce'a od teraz mozna liczyc dalej
          key_state_interakcja = BUTTON_IS_HOLDED;
        }
        // jesli po tym czasie przycisk jest zwolniony to uzytkownik byl w stanie tak szybko kliknac
        // jednakze to oznacza ze nastepuje kolejn stan tj. debounce od releasa i zapisujemy po prostu
        // czas debounca, troche zaklamanie ale bardzo znikle
        else
        {
          *hnr_time_ptr++ = ISR_DEBOUNCE_CNT;
          button_state_time = 0;
          saved_states++;
          keycnt2 = ISR_DEBOUNCE_CNT;
          key_state_interakcja = BUTTON_NOT_PRESSED;
        }
      }

      // jesli stan, ze przycisk wcisniety oraz wciaz jest wciskany dodawaj co przerwanie czas
      if (key_state_interakcja == BUTTON_IS_HOLDED)
      {
        button_state_time++;

        // jesli stan wcisniecia przycisku oraz przycisk zostal zwolniony, czas na debounce release'a przycisku
        // zapisz czas wcisniecia, wyzeruj licznik i przejdz do liczenia zwolnienia przycisku
        if (ACTION_KEY_VAL)
        {
          *hnr_time_ptr++ = button_state_time;
          button_state_time = 0;
          saved_states++;
          keycnt2 = ISR_DEBOUNCE_CNT;
          key_state_interakcja = BUTTON_NOT_PRESSED;
        }
      }

      // jesli przycisk nie wcisniety oraz key_state nie wcisniety i czas debounce 0  to dodawaj
      // czas release, tak bedzie sie dzialo od razu po przejsciu w stan w device state, jednakze
      // zapis odbywa sie dopiero po pierwszym wcisnieciu "holdzie" przycisku
      if (key_state_interakcja == BUTTON_NOT_PRESSED)
      {
        if (ACTION_KEY_VAL)
        {
          button_state_time = ISR_DEBOUNCE_CNT;  // dodanie czasu debounce'a od teraz mozna liczyc dalej
          key_state_interakcja = BUTTON_IS_RELEASED;
        }
        else
        {
          *hnr_time_ptr++ = ISR_DEBOUNCE_CNT;
          button_state_time = 0;
          saved_states++;
          keycnt2 = ISR_DEBOUNCE_CNT;
          key_state_interakcja = BUTTON_PRESSED;
        }
      }
      // przycisk zotal puszczony zliczaj czas przerwy i zapisz do tablicy po pojawieniu sie stanu
      // wcisniecia przycisku ponownie
      if (key_state_interakcja == BUTTON_IS_RELEASED)
      {
        button_state_time++;

        // jesli wcisnieto podczas zwolnionego przycisku, czyli liczymy kolejny stan wcisniecia
        if (!ACTION_KEY_VAL)
        {
          *hnr_time_ptr++ = button_state_time;
          button_state_time = 0;
          saved_states++;
          keycnt2 = ISR_DEBOUNCE_CNT;
          key_state_interakcja = BUTTON_PRESSED;
        }
      }
    }


    // jesli zapisano piec stanow 3H i 2R to zakoncz i przejdz do oceny
    if (saved_states >= NUM_ACTIONS)
    {

      // przed pierwszym holdem wyswietl liczbe kolejnej odebranej interakcji
      if(index == 0)
      {
        button_state_time = 0;
        keycnt2 = _BREAK_TIME_S(2);  // przerwa 2 s tak ad hoc min miedzy kolejnymi interakcjami
        hnr_time_ptr -= NUM_ACTIONS;
        #if DEBUG_STATE == _ON
        StrToSerial("Interakcja nr:");
        how_many_times_sent++;
        PutUInt8ToSerial(how_many_times_sent);
        StrToSerial("\n");
        #endif
      }

      #if DEBUG_STATE == _ON
      if (!(index % 2))
        StrToSerial("H"); // H - Hold
      else
        StrToSerial("R"); // R - Release

      PutUInt8ToSerial(index);
      StrToSerial(": ");

      // dziele przez 5 bo przerwanie jest co 0,2ms czyli 2/10 => 1/5 w taki sposob uzyskuje wartosc
      // w ms
      PutUInt16ToSerial(*hnr_time_ptr / 5, TRUE, 5);
      // kasuje wartosc, przygotwuje na kolejna interakcje
      StrToSerial(" ms\n");
      #endif

      *hnr_time_ptr = 0;
      hnr_time_ptr++;
      index++;
    }
  }

  if(index >= NUM_ACTIONS)
  {
    key_state_interakcja = INITIAL_KEY_STATE;
    saved_states = 0;
    index = 0;
    hnr_time_ptr -= NUM_ACTIONS;
  }

  if (keycnt2 > 0)
    keycnt2--;

  if(device_state == ST_OCENA)
  {
//    activity_rate = EstimateActivity(holdandreleasetime, random_values, activity_rate);
  }
}

//// function to estimate activity of user of device
//static uint8 EstimateActivity(uint16 hnr_time[], uchar rnd_values[], uint8 current_activity)
//{
//  // there is a measure time of interaction and ive got random time
//  // extend random values to interaction resolution
//  // substract them and change activity and then finally change device state
//  return current_activity;
//}
