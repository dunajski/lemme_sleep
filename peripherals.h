/*
 * peripherals.h
 *
 *  Created on: 17 sty 2019
 *      Author: Dunajski
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

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

#define STATE_LED_VAL ((TPortABits *)&PINA)->state_led
#define STATE_LED_DIR ((TPortABits *)&DDRA)->state_led
#define STATE_LED_OUT ((TPortABits *)&PORTA)->state_led

#define DEBUG_LED_VAL ((TPortABits *)&PINA)->debug_led
#define DEBUG_LED_DIR  ((TPortABits *)&DDRA)->debug_led
#define DEBUG_LED_OUT ((TPortABits *)&PORTA)->debug_led

#define MOTOR_VAL ((TPortABits *)&PINA)->motor
#define MOTOR_DIR ((TPortABits *)&DDRA)->motor
#define MOTOR_OUT ((TPortABits *)&PORTA)->motor

#define ACTION_KEY_VAL ((TPortABits *)&PINA)->action_key
#define ACTION_KEY_DIR  ((TPortABits *)&DDRA)->action_key
#define ACTION_KEY_PULLUP ((TPortABits *)&PORTA)->action_key

#define ADC_PIN_VAL ((TPortABits *)&PINA)->adc_pin
#define ADC_PIN_DIR ((TPortABits *)&DDRA)->adc_pin
#define ADC_PIN_PULLUP ((TPortABits *)&PORTA)->adc_pin

#define DEBUG_LED_ON DEBUG_LED_OUT=1
#define DEBUG_LED_OFF DEBUG_LED_OUT=0


void InitUart(void);
void InitTimer0(void);
void InitTimer2(void);
void InitAdc(void);
void InitIO(void);

#endif /* PERIPHERALS_H_ */
