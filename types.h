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


#endif /* TYPES_H_ */
