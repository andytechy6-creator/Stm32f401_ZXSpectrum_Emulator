/*
 * spectrum.h
 *
 *  Created on: Jan 31, 2026
 *      Author: Andy Nicholson
 */

#ifndef INC_SPECTRUM_H_
#define INC_SPECTRUM_H_


#include <stdint.h>
#include "spectrum_core.h"

// Spectrum vertical window inside the VGA frame (logical lines)
#define SPEC_Y_START   40
#define SPEC_Y_END     (SPEC_Y_START + 192)

// Border colour (0–7, Spectrum ink/paper index)
extern uint8_t spectrum_border_colour;

// Pointer to Spectrum RAM (at least 0x5800 bytes from 0x4000)
// extern uint8_t *spectrum_ram;

// Public API
void spectrum_render_scanline(uint16_t y, uint8_t *dst);



#endif /* INC_SPECTRUM_H_ */
