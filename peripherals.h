/*
 * peripherals.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "communication.h"
#include "random.h"
#include "types.h"

#define NUM_RND     13
#define MAX_NUM_ACTIONS  5

typedef struct PortABits
{
  volatile uchar adc_pin :1;    // PA 0 losowanie
  volatile uchar pwr_adc :1;    // PA 1 mierzenie napiecia zas.
  volatile uchar :4;            // PA 2-5
  volatile uchar debug_led :1;  // PA 6 // do prototypowania nie beda lutowane
  volatile uchar state_led :1;  // PA 7 // do prototypowania nie beda lutowane
} TPortABits;

typedef struct PortDBits
{
  volatile uchar :2;            // PD 0-1
  volatile uchar action_lever:1;// PD 2
  volatile uchar :2;            // PD 3-4
  volatile uchar emmiter :1;    // PD 5 (OCR1A)
  volatile uchar motor :1;      // PD 6
  volatile uchar :1;            // PD 7
} TPortDBits;

#define STATE_LED_VAL ((TPortABits *)&PINA)->state_led
#define STATE_LED_DIR ((TPortABits *)&DDRA)->state_led
#define STATE_LED_OUT ((TPortABits *)&PORTA)->state_led

#define DEBUG_LED_VAL ((TPortABits *)&PINA)->debug_led
#define DEBUG_LED_DIR ((TPortABits *)&DDRA)->debug_led
#define DEBUG_LED_OUT ((TPortABits *)&PORTA)->debug_led

#define ADC_PIN_VAL     ((TPortABits *)&PINA)->adc_pin
#define ADC_PIN_DIR     ((TPortABits *)&DDRA)->adc_pin
#define ADC_PIN_PULLUP  ((TPortABits *)&PORTA)->adc_pin

#define PWR_ADC_VAL     ((TPortABits *)&PINA)->pwr_adc
#define PWR_ADC_DIR     ((TPortABits *)&DDRA)->pwr_adc
#define PWR_ADC_PULLUP  ((TPortABits *)&PORTA)->pwr_adc

#define MOTOR_VAL ((TPortDBits *)&PIND)->motor
#define MOTOR_DIR ((TPortDBits *)&DDRD)->motor
#define MOTOR_OUT ((TPortDBits *)&PORTD)->motor

#define EMMITER_VAL ((TPortDBits *)&PIND)->emmiter
#define EMMITER_DIR ((TPortDBits *)&DDRD)->emmiter
#define EMMITER_OUT ((TPortDBits *)&PORTD)->emmiter

#define LEVER_VAL    ((TPortDBits *)&PIND)->action_lever
#define LEVER_DIR    ((TPortDBits *)&DDRD)->action_lever
#define LEVER_PULLUP ((TPortDBits *)&PORTD)->action_lever

#define DEBUG_LED_ON   (DEBUG_LED_OUT=1)
#define DEBUG_LED_OFF  (DEBUG_LED_OUT=0)
#define DEBUG_LED_TOGGLE  (DEBUG_LED_OUT=(!DEBUG_LED_OUT))

#define STATE_LED_ON   (STATE_LED_OUT=1)
#define STATE_LED_OFF  (STATE_LED_OUT=0)
#define STATE_LED_TOGGLE  (STATE_LED_OUT=(!STATE_LED_OUT))

#define MOTOR_ON   (MOTOR_OUT=0) // PNP bramka sterowana '0'
#define MOTOR_OFF  (MOTOR_OUT=1) // PNP bramka sterowana '0'

#define LEVER_PRESSED   (!LEVER_VAL)
#define LEVER_UNPRESSED (LEVER_VAL)

typedef struct
{
  uint16 hnr_time[MAX_NUM_ACTIONS];
  uint16 rnd_time[MAX_NUM_ACTIONS];
  uint16 extended_user_seq[MAX_NUM_ACTIONS];
  int16 diff_time[MAX_NUM_ACTIONS];
  uint32 whole_user_sequence;
  uint32 whole_random_sequence;
  uint32 whole_extended_user_seq;
  uint8 current_act;
} TLastSequence;

extern volatile TLastSequence Sequence;

typedef struct
{
  uint8 fails_in_row;
  uint8 num_actions;
  uint8 delay_between_sequences_s;
} TSequenceParameters;

extern volatile TSequenceParameters Seq_params;

/*
 *******************************************************************************
 * Funkcja do zmiany wartosci 16bitowej w bloku, uniemozliwiajacym nadpisanie
 * przez inne operacje. Do dzialania wymaga biblioteki util/atomic.h.
 * Operacja atomic wywolywana jest z przywroceniem stanu SREG (RestoreON).
 * [in] uint16 var_to_set - wskaznik na rejestr/zmienna do ustawienia
 * [in] uint16 value - wartosc do wpisania do rejestru/zmiennej
 *******************************************************************************
 */
void SetUint16_atomic(volatile uint16 * var_to_set, uint16 value);

/*
 *******************************************************************************
 * Inicjuje UART, 9600/8N1. Komunikacja na przerwaniach.
 *******************************************************************************
 */
void InitUsart(void);

/*
 *******************************************************************************
 * Inicjalizacja Timer0 do sterowania silnikiem (zalaczanie i rozlaczanie)
 * 10 ms CTC/presc. 1024.
 *******************************************************************************
 */
void InitTimer0(void);

/*
 *******************************************************************************
 * Inicjalizacja Timer1 do sterowania silnikiem (PWM, do sterowania zasilaniem
 * silnika, zaleznie od napiecia zasilania). Sterowanie wyjsciem OC1A (inverted
 * mode, bo uzywam PNP tranzystora do zalaczania silnika) Fast PWM/presc. 64.
 *******************************************************************************
 */
void InitTimer1(void);

/*
 *******************************************************************************
 * Inicjalizacja Timer2 do obslugi stanow oraz przycisku, 0,2 ms CTC/presc. 8.
 *******************************************************************************
 */
void InitTimer2(void);

/*
 *******************************************************************************
 * Inicjalizacja przetwornika do losowania zmiennych. ADC0, prescaler 64,
 * odniesienia na AVCC.
 *******************************************************************************
 */
void InitAdc(void);

/*
 *******************************************************************************
 * Inicjalizacja wejsc/wyjsc MCU. Tutaj ustawiam LEDy i stan poczatkowy appki.
 *******************************************************************************
 */
void InitIOs(void);

/*
 *******************************************************************************
 * Przeprowadza procedure usypiania MCU, wylaczajac przerwania od wszystkich
 * peryferiow oprocz przerwania zewnetrznego. Nastepnie po przebudzeniu
 * przywraca wszystkie przerwania od peryferiow.
 *******************************************************************************
 */
void GoToSleep(void);

/*
 *******************************************************************************
 * Inicjalizacja External Interrupt 0 do wybudzania MCU. Stan niski na PD2.
 *******************************************************************************
 */
void InitExternalInterupt1(void);

/*
 *******************************************************************************
 * Ustawia delay w Timerze2 w sekundach
 *******************************************************************************
 */
void DelayandSetNextState(uint8 delay_s, uint8 next_state);

#endif /* PERIPHERALS_H_ */
