/*
 * peripherals.c
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#include <avr/interrupt.h>
#include <avr/iom32.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <string.h>

#include <stdint.h>
#include "peripherals.h"
#include "energy.h"


#define TRUE (1)
#define FALSE (0)

volatile uint16 * hnr_time_ptr = holdandreleasetime;

static uint8 EstimateActivity(volatile uint16 hnr_time[],volatile uchar rnd_values[], uint8 current_activity);

// stany przycisku podczas odliczania czasu wcisnieta i puszczenia dla stanu ST_INTERAKCJA
typedef enum
{
  INITIAL_KEY_STATE = 0,    // stan przed wcisnieciem pierwszym
  BUTTON_NOT_PRESSED,       // stan puszczenia, ale musi uprzednio wystapic choc jedno wcisniecie
  BUTTON_PRESSED,           // stan do rozpoczecia odlcizania wcisniecia, oraz ogolnego rozpoczenia mierzenia
  BUTTON_IS_HOLDED,         // stan do naliczania czasu po debounce po wcisnieciu przycisku
  BUTTON_IS_RELEASED,       // stan do naliczania czasu po debounce po zwolnieniu przycisku
  BUTTON_HOLDED_TOO_LONG,   // stan po przekroczeniu maks czasu wcisniecia (10 s)
  BUTTON_RELEASED_TOO_LONG  // stan po przekroczeniu maks czasu puszczenia (10 s)
} TKeyInterakcjastates;

/*
 *******************************************************************************
 * Przeprowadza procedure usypiania MCU, wylaczajac przerwania od wszystkich
 * peryferiow oprocz przerwania zewnetrznego. Nastepnie po przebudzeniu
 * przywraca wszystkie przerwania od peryferiow.
 *******************************************************************************
 */
void GoToSleep(void)
{
  // Wylaczenie przerwan od innych peryferiow, zeby moc wybudzac tylko EXT0
  TIMSK  &= ~(1 << OCIE0);
  TIMSK  &= ~(1 << OCIE2);
  ADCSRA &= ~(1 <<  ADIE);
  UCSRB  &= ~(1 << RXCIE);
  UCSRB  &= ~(1 << UDRIE);

  STATE_LED_ON;

  // Zezwol na przerwanie od EXT0 zeby moc wybudzic
  GICR |= (1 << INT0);
  sei();

  // Ustaw tryb oszczedzania energii, na ten moment IDLE dla testow ale moga byc
  // inne. MCU zatrzymuje sie tutaj na sleep_cpu(); i po wybudzeniu kontynuuje
  // od miejsca sleep_disable();
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_cpu();
  sleep_disable();

  STATE_LED_OFF;
  // Nastepuje wybudzenie, czas normalnej pracy, a wiec potrzebujemy tych
  // wszystkich przerwan
  TIMSK  |= (1 << OCIE0);
  TIMSK  |= (1 << OCIE2);
  ADCSRA |= (1 <<  ADIE);
  UCSRB  |= (1 << RXCIE);
  UCSRB  |= (1 << UDRIE);
  sei();
}

/*
 *******************************************************************************
 * Funkcja do zmiany wartosci 16bitowej w bloku, uniemozliwiajacym nadpisanie
 * przez inne operacje. Do dzialania wymaga biblioteki util/atomic.h.
 * Operacja atomic wywolywana jest z przywroceniem stanu SREG (RestoreON).
 * [in] uint16 var_to_set - wskaznik na rejestr/zmienna do ustawienia
 * [in] uint16 value - wartosc do wpisania do rejestru/zmiennej
 *******************************************************************************
 */
void SetUint16_atomic(volatile uint16 * var_to_set, uint16 value)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    *var_to_set = value;
  }
}

#define BAUDRATE 9600UL
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1)

/*
 *******************************************************************************
 * Inicjuje UART, 9600/8N1. Komunikacja na przerwaniach.
 *******************************************************************************
 */
void InitUsart(void)
{
  // Baudrate 9600
  UBRRH = (BAUD_REG >> 8);
  UBRRL = BAUD_REG;
  // Format ramki danych 8N1
  // URSEL musi byc '1' zeby pisac do UCSRC
  UCSRC |= ((1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1));
  UCSRB |= ((1 << RXEN) | (1 << TXEN));  // RX/TX na pinach PD0/PD1
  UCSRB |= (1 << RXCIE);                 // RX ISR enable
}

/*
 *******************************************************************************
 * Inicjalizacja Timer0 do sterowania silnikiem (zalaczanie i rozlaczanie)
 * 10 ms CTC/presc. 1024.
 *******************************************************************************
 */
void InitTimer0(void)
{
  // ISR execute period 10 ms / interrupt /  CTC   // f CTC = fio/(presc*(1+OCR)
  // -> t = 10 ms 1/t = fCTC -> 1/10ms -> 100 Hz
  TCCR0 |= (1 << WGM01) | (1 << CS02) | (1 << CS00);  // prescaler 1024  | CTC mode
  TIMSK |= (1 << OCIE0);                              //ctc timer0 isr enable
  OCR0 = 77;
}

/*
 *******************************************************************************
 * Inicjalizacja Timer1 do sterowania silnikiem (PWM, do sterowania zasilaniem
 * silnika, zaleznie od napiecia zasilania). Sterowanie wyjsciem OC1A (inverted
 * mode, bo uzywam PNP tranzystora do zalaczania silnika) Fast PWM/presc. 64.
 *******************************************************************************
 */
void InitTimer1(void)
{
  // Fast PWm// f PWM = fio/(presc*(1+OCR)
//  TCCR1A |= (1 << COM1A1) | (1 << COM1A0);  // Inverted mode ('1' on match) // MOTOR
 // NOTE: uwaga na stan wyjscia PWM
  TCCR1A |= (1 << COM1A1);  // Inverted mode ('1' on match) // LED
  TCCR1A |= (1 << WGM10);                   // Fast PWM
  TCCR1B |= (1 << WGM12);                   // Fast PWM
  TCCR1B |= (1 << CS11);                    // Prescaler 8

  SetUint16_atomic(&OCR1A, 0x0000);
}

/*
 *******************************************************************************
 * Inicjalizacja Timer2 do obslugi stanow oraz przycisku, 0,2 ms CTC/presc. 8.
 *******************************************************************************
 */
void InitTimer2(void)
{
  OCR2 = 199;  // F_CPU 8MHz przerwanie co 0,2ms f CTC = fio/(presc*(1+OCR))
  // -> f CTC = 8 000 000 / (8 * 200) = 5000 => 1/5000 = 0,2 ms
  TCCR2 |= (1 << WGM21 | 1 << CS21);  //CTC | presc 8
  TIMSK |= (1 << OCIE2);              //CTC timer2 isr enable
}

/*
 *******************************************************************************
 * Inicjalizacja External Interrupt 0 do wybudzania MCU. Stan niski na PD2.
 *******************************************************************************
 */
void InitExternalInterupt1(void)
{
  // level interrupt INT0 (low level)
  MCUCR &= ~((1 << ISC01) | (1 << ISC00));
  GICR  |= INT0; // interrupt na zdarzenie na wejsciu INT1
}

/*
 *******************************************************************************
 * Inicjalizacja przetwornika do losowania zmiennych. ADC0, prescaler 64,
 * odniesienia na AVCC.
 *******************************************************************************
 */
void InitAdc(void)
{
  ADMUX |= (1 << REFS0);  //drifting pin lepiej na AREFie bez niczego
//  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
  ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS2);
  // ADC ENABLE/start conversion/autotriger EN/interrupt execute EN/ presk 64  f_adc=8MHz/64=125kHz
}

/*
 *******************************************************************************
 * Inicjalizacja wejsc/wyjsc MCU. Tutaj ustawiam LEDy i stan poczatkowy appki.
 *******************************************************************************
 */
void InitIOs(void)
{
  STATE_LED_DIR = 1; // dioda testowa
  DEBUG_LED_DIR = 1; // dioda testowa

  MOTOR_DIR = 1;

  LEVER_PULLUP = 1;
  LEVER_DIR    = 0;

  ADC_PIN_DIR    = 0; // wejscie
  ADC_PIN_PULLUP = 0; // bez pullupu, niech dryfuje

  PWR_ADC_DIR     = 0;
  PWR_ADC_PULLUP  = 1;

  device_state = ST_POWER_DWN; // ropoczynamy od stanu uspienia
}

#define TIME_10SECS (50000UL) // 0,2 ms * 50 000 = 10 sekund
#define TIME_120SECS (600000UL) // 0,2 ms * 600 000 = 120 sekund
#define TIME_20SECS (100000UL)
#define PRESS_TO_WAKE_UP_COUNT (3)
#define ISR_DEBOUNCE_CNT (400) // 200 * 0,2 ms = 40 ms

/*
 *******************************************************************************
 * Przerwanie Timer2 do obslugi klawisza i calej maszyny stanow aplikacji.
 *******************************************************************************
 */
ISR(TIMER2_COMP_vect)
{
  // Obsluga wcisniec przyciskiem
  static uint8  keyr = 0;
  static uint8  keylev = 0;
  static uint16 keycnt = 0;
  // Obsluga wcisniec przyciskiem dla stanu ST_INTERAKCJA
  static uint8  index = 0;
  static uint8  saved_states = 0;
  static uint8  key_state_interakcja = INITIAL_KEY_STATE;
  static uint16 keycnt2 = 0;
  static uint16 button_state_time = 0; // 0,2 ms * 0xFFFF = 13 s (max czasu)
  static uint32 sequence_time = 0; // zeby sprawdzac maksymalny czas interakcji
  // Obsluga wybudzania
  static uint16 wake_up_timer = 0;
  static uint8  wake_up_cnt = 0;

  static uint8  how_many_times_sent = 0;
  static uint8  activity_rate;

  static uint16 goto_sleep_delay = 0; // test

  // Obsluga przycisku dla wszystkich stanow
  if (keycnt == 0)
  {
    switch (keylev)
    {
      case 0:  // waiting for press
        if (LEVER_PRESSED)
        {
          keyr = LEVER_VAL;
          keylev = 1;
          keycnt = ISR_DEBOUNCE_CNT;
        }
      break;

      case 1: // pressed,debounced
        if (LEVER_VAL == keyr)
        {
          if (device_state != ST_INTERAKCJA)
            keylev = 2;

          switch (device_state)
          {
            case ST_WAIT_TO_WAKE_UP:
              wake_up_cnt++;
              keylev = 2;
            break;

            case ST_INTERAKCJA:
              // odbieranie sekwencji od uzytkownika w inny miejscu
            break;

            case ST_POWER_DWN:
            case ST_WIBROWANIE: // w trakcie wibrowania nie obchodzi nas czy ktos klika
            case ST_OCENA:
            case ST_LOSOWANIE:
            default:
            break;
          }
        }
        // jesli stan sie rozni
        else
          keylev = 0;
      break;

      case 2:
        if (LEVER_VAL == 1)
          keylev = 0;
      break;
    }
  }

  if (keycnt > 0)
    keycnt--;
  //============================================================================

  // Obsluga stanu ST_WAIT_TO_WAKE_UP po wybudzeniu
  if (device_state == ST_WAIT_TO_WAKE_UP)
  {
    if (wake_up_timer >= TIME_10SECS)
    {
      wake_up_timer = 0;
      wake_up_cnt = 0;
      device_state = ST_POWER_DWN;
    }

    if (wake_up_cnt >= PRESS_TO_WAKE_UP_COUNT)
    {
      wake_up_cnt = 0;
      device_state = ST_LOSOWANIE;
    }

    wake_up_timer++;
  }
  //============================================================================


  // Obsluga stanu LOSOWANIE jesli poprwanie wybudzono i przed kazda interakcja
  if (device_state == ST_LOSOWANIE && change_random == 0)
  {
    TurnADCOn;
    change_random = 1;
  }
  //============================================================================

  // Obsluga stanu MIERZENIE_ZASILANIA
  if (device_state == ST_MIERZENIE_ZASILANIA)
  {
    SetAdcToMeasureSupplVoltage();
    TurnADCOn;
  }
  //============================================================================

  // Obsluga stanu INTERAKCJA po wylosowaniu i wibrowaniu silnika
  if (device_state == ST_INTERAKCJA)
  {
    if (key_state_interakcja != INITIAL_KEY_STATE)
      sequence_time++;

    // jesli czas sekewencji przekroczono wpisz do wszystkich
    // maks dlugosci i potraktuj jako koniec sekwencji
    if (sequence_time >= TIME_20SECS)
    {
      // TODO: zastanwow sie co wtedy
    }

    if (!keycnt2 && (saved_states < NUM_ACTIONS))
    {
      // aby rozpoczac mierzenie hold and release musi byc stan init, zeby mozna bylo rozpoczac
      // mierzenie od wcisniecia, a wiec oczekuje na wcisniecie przycisku
      if ((key_state_interakcja == INITIAL_KEY_STATE) && (LEVER_PRESSED))
      {
        // debouncing wcisnieto przycisk
        keycnt2 = ISR_DEBOUNCE_CNT;
        key_state_interakcja = BUTTON_PRESSED;
      }

      // sprawdzamy czy po debounce przycisk wciaz jest przycisniety
      if (key_state_interakcja == BUTTON_PRESSED)
      {
        if (LEVER_PRESSED)
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

        // jesli ktos wcisnal dluzej niz 10 sec
        if (button_state_time >= TIME_10SECS)
        {
          #if DEBUG_STATE
          StrToSerial("Przegles z tym holdem\n");
          #endif
          key_state_interakcja = BUTTON_HOLDED_TOO_LONG;
          *hnr_time_ptr++ = 0xFFFF;
        }

        // jesli stan wcisniecia przycisku oraz przycisk zostal zwolniony, czas na debounce release'a przycisku
        // zapisz czas wcisniecia, wyzeruj licznik i przejdz do liczenia zwolnienia przycisku
        if (LEVER_UNPRESSED)
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
        if (LEVER_UNPRESSED)
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

        // jesli ktos puscil na  dluzej niz 10 sec
        if (button_state_time >= TIME_10SECS)
        {
          #if DEBUG_STATE
          StrToSerial("Przegles z ta przerwa\n");
          #endif
          key_state_interakcja = BUTTON_RELEASED_TOO_LONG;
          *hnr_time_ptr++ = 0xFFFF;
        }

        // jesli wcisnieto podczas zwolnionego przycisku, czyli liczymy kolejny stan wcisniecia
        if (LEVER_PRESSED)
        {
          *hnr_time_ptr++ = button_state_time;
          button_state_time = 0;
          saved_states++;
          keycnt2 = ISR_DEBOUNCE_CNT;
          key_state_interakcja = BUTTON_PRESSED;
        }
      }
    }

    if (key_state_interakcja == BUTTON_HOLDED_TOO_LONG)
    {
      // odpusczono po dlugim wcisnieciu
      if (LEVER_UNPRESSED)
      {
        #if DEBUG_STATE
        StrToSerial("...w koncu puscil\n");
        #endif
        keycnt2 = ISR_DEBOUNCE_CNT;
        key_state_interakcja = BUTTON_NOT_PRESSED;
        button_state_time = 0;
        saved_states++;
      }
    }
    else if (key_state_interakcja == BUTTON_RELEASED_TOO_LONG)
    {
      // wcisnieto po dlugiej przerwie
      if (LEVER_PRESSED)
      {
        #if DEBUG_STATE
        StrToSerial("...w koncu wcisnal\n");
        #endif
        keycnt2 = ISR_DEBOUNCE_CNT;
        key_state_interakcja = BUTTON_PRESSED;
        button_state_time = 0;
        saved_states++;
      }
    }

    // jesli zapisano piec stanow 3H i 2R to zakoncz i przejdz do oceny
    if (saved_states >= NUM_ACTIONS)
    {

      // TODO przekopiuj do tablic i ustaw ocene
      // wyzeruj tablice robocza

      // przed pierwszym holdem wyswietl liczbe kolejnej odebranej interakcji
      if (index == 0)
      {
        button_state_time = 0;
        // przesun na poczatek
        hnr_time_ptr= holdandreleasetime;
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

  if (index >= NUM_ACTIONS)
  {
    key_state_interakcja = INITIAL_KEY_STATE;
    saved_states = 0;
    index = 0;
    // przesun na poczatek
    hnr_time_ptr = holdandreleasetime;
    device_state = ST_OCENA;
    sequence_time = 0;
  }

  if (keycnt2 > 0)
    keycnt2--;

  }
  //============================================================================

  // Obsluga stanu INTERAKCJA po wylosowaniu i wibrowaniu silnika
  if (device_state == ST_OCENA)
  {
    goto_sleep_delay++;
    if (goto_sleep_delay == 10)
    {
      activity_rate = EstimateActivity(holdandreleasetime, random_values, activity_rate);
      #if DEBUG_STATE == _ON
      StrToSerial("Aktualna aktywnosc:");
      PutUInt8ToSerial(activity_rate);
      StrToSerial("\nOcena niezaimplementowana, usypianie\n");
      #endif
    }

    // opoznij przejscie o 2,5 s zeby wszystko sie wyslalo
    if (goto_sleep_delay >= 5000)
    {
      goto_sleep_delay = 0;
      device_state = ST_POWER_DWN;
    }
  }
  //============================================================================

}

// TODO: ponizsze do zrobienia
/*
 *******************************************************************************
 * Ocenia aktywnosc uzytkownika na podstawe roznic pomiedzy interakcja, losowa
 * probka z przetwornika i poprzednia aktywnoscia. Po wyliczeniu,
 * czyscic obie tablice.
 * [in] uint16[] hnr_time - wskaznik na tablice z wynikami hold and release
 * [in] uint16[] rnd_values - wskaznik na tablice z wynikami losowania
 * [in] uint8 current_activity - aktywnosc z uwzglednieniem poprzednich akcji
 * [out] uint8 - estymowana aktywnosc
 *******************************************************************************
 */
static uint8 EstimateActivity(volatile uint16 hnr_time[], volatile uchar rnd_values[], uint8 current_activity)
{
  // there is a measure time of interaction and ive got random time
  // extend random values to interaction resolution
  // substract them and change activity and then finally change device state
  return current_activity;
}
