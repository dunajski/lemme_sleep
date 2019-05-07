/*
 * types.h
 *
 *  Created on: 16 sty 2019
 *      Author: Dunajski
 */

#ifndef TYPES_H_
#define TYPES_H_

#define BV(bit) (1 << bit)
#define SET_BIT(byte, bit) (byte |= BV(bit))
#define CLEAR_BIT(byte, bit) (byte &= ~BV(bit))
#define TOGGLE_BIT(byte, bit) (byte ^= BV(bit))

#define _NumItems(array) (sizeof(array)/sizeof(array[0]))

typedef enum DeviceState
{
  ST_IDLE = 0,
  ST_LOSOWANIE,   // tuz przed nadawaniem krotki okres przeznacozny na wylosowanie
  ST_WIBROWANIE,
  ST_INTERAKCJA,  // rejestrowanie odpowiedzi uzytkownika
  ST_OCENA        // ocena "jakosci" odzworowania sygnalu losowego
} TDeviceStates;

typedef unsigned char uchar;
typedef uint16_t uint16;
typedef uint8_t uint8;
typedef int32_t int32;

#endif /* TYPES_H_ */
