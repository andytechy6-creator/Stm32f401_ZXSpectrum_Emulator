/*
 * z80.h
 *
 *  Created on: Feb 1, 2026
 *      Author: Andy Nicholson
 */

#ifndef INC_Z80_H_
#define INC_Z80_H_


#include <stdint.h>

typedef struct {
    uint16_t pc, sp;
    uint16_t af, bc, de, hl;
    uint16_t af2, bc2, de2, hl2;
    uint16_t ix, iy;
    uint8_t  iff1, iff2;
    uint8_t  im;
} z80_t;

extern z80_t z80;

void z80_reset(void);
void z80_step(void);


#endif /* INC_Z80_H_ */
