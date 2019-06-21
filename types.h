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

// macro zeby nie wywalac duzo kodu, to co w trakcie prototypowania debugowania jest uzywane mozna
// tym zakomentowac
#define DEBUG_STATE _ON // _ON/_OFF
#define _ON         (1)
#define _OFF        (2)

#define _NumItems(array) (sizeof(array)/sizeof(array[0]))

typedef enum
{
  ST_POWER_DWN = 0,
  ST_WAIT_TO_WAKE_UP,
  ST_LOSOWANIE,   // tuz przed nadawaniem krotki okres przeznacozny na wylosowanie
  ST_WIBROWANIE,
  ST_INTERAKCJA,  // rejestrowanie odpowiedzi uzytkownika
  ST_OCENA        // ocena "jakosci" odzworowania sygnalu losowego
} TDeviceStates;

typedef unsigned char uchar;
typedef uint16_t uint16;
typedef uint8_t uint8;
typedef int32_t int32;
typedef uint32_t uint32;

#endif /* TYPES_H_ */
