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

#define BAUDRATE 9600L//115200L
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) //dzielnika cz. UBRR

#endif /* TYPES_H_ */
