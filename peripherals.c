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
#define TRUE (1)
#define FALSE (0)

volatile uint16 * hnr_time_ptr = holdandreleasetime;
volatile uchar how_many_times_sent = 0;

// stany przycisku podczas odliczania czasu wcisnieta i puszczenia dla stanu ST_INTERAKCJA
typedef enum KeySubState
{
  INITIAL_KEY_STATE = 0, // stan przed wcisnieciem pierwszym
  BUTTON_NOT_PRESSED, // stan puszczenia, ale musi uprzednio wystapic choc jedno wcisniecie
  BUTTON_PRESSED, // stan do rozpoczecia odlcizania wcisniecia, oraz ogolnego rozpoczenia mierzenia
  BUTTON_IS_HOLDED, // stan do naliczania czasu po debounce po wcisnieciu przycisku
  BUTTON_IS_RELEASED // stan do naliczania czasu po debounce po zwolnieniu przycisku

} TKeySubstates;

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

  device_state = ST_INTERAKCJA; // na razie rozpocznij od razu od losowania

//  PutUint16ToSerial(0xFF98);
//  StrToSerial("\n");
//  PutUint16ToSerial(0x0001);
//  StrToSerial("\n");
//  PutUint16ToSerial(0x02);
//  StrToSerial("\n");
//  PutUint16ToSerial(0x0F0F);
//  StrToSerial("\n");
//  PutUint16ToSerial(0x0FFF);
//  StrToSerial("\n");
}

// isr to debounce key and measure feedback
//ISR 0,4ms
// keycnt 200 then 0,4 * 200 = 80ms
// 0,2ms?
ISR(TIMER2_COMP_vect)
{
  static uint16 keycnt = 0, keycnt2 = 0;
  static uint8 keylev = 0;
  static uint8 keyr = 0;
  static uint16 change_random_cnt = 0;
  static uint16 button_state_time = 0; // powinnien 16 bitowy wystarczyc 0,4 ms x 0xFFFF = ~26 s
  static uint8 key_state_interakcja = INITIAL_KEY_STATE;
  // pomocznia zmienna pewnie potem do wyciecia, zeby przechowywac ilosc zmian stanu
  static uint8 saved_states = 0;
  static int m = 0;
  // ISR co 0,4ms co tyle, losujemy random lsb
  // zeby nie "zapchac" kanalu trasnmisyjnego
  // stad volatile od wysylania bedzie zmieniany tutaj
  // jeszcze od state bedzie zmieniane, jednakze na razie jest tak

  change_random_cnt++;

  if (change_random_cnt >= 25000) // co 200ms losowanie kolejnej liczby
  {
    change_random = 1;

    if (device_state != ST_OCENA)
    {
      TOGGLE_BIT(PORTA,PA6);
    }
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
              // jesli uzytkownik puscil przycisk to przestan naliczac button_state_time i zacznij
              // debounce dla puszczenia przycisku
            break;
            case ST_OCENA:
              // na razie wyslij po odliczaniu wartosci zobaczymy czy naliczanie ma sens
//              for (int i = 0; i <= 4 ; i++)
//              {
//                PutUint16ToSerial(holdandreleasetime[i]);
//              }
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

      if(m == 0)
      {
        button_state_time = 0;
        keycnt2 = 10000;  // przerwa 2 s
//        hnr_time_ptr = holdandreleasetime;
        hnr_time_ptr -= NUM_ACTIONS;
        //StrToSerial("Losowanie nr:");
        how_many_times_sent++;
        PutUInt8ToSerial(how_many_times_sent);
        StrToSerial("\n");
      }

      if (!(m % 2))
        StrToSerial("H");
      else
        StrToSerial("R");

      PutUInt8ToSerial(m);
      StrToSerial(": ");

      PutUint16ToSerial(*hnr_time_ptr / 5, TRUE, 4);
      *hnr_time_ptr = 0;
      StrToSerial(" ms\n");
      hnr_time_ptr++;
      m++;
    }
  }

  if(m >= NUM_ACTIONS)
  {
    key_state_interakcja = INITIAL_KEY_STATE;
    saved_states = 0;
    m = 0;
    hnr_time_ptr -= NUM_ACTIONS;
  }

  if (keycnt2 > 0)
    keycnt2--;
}

