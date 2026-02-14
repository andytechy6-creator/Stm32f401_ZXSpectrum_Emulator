/*
 * vga.h
 *
 *  Created on: Jan 31, 2026
 *      Author: Andy Nicholson
 */

#ifndef INC_VGA_H_
#define INC_VGA_H_

#include "vga_timing.h"

#include <stdint.h>

#define VGA_PACK_RGB(r, g, b)  ((uint32_t)(((r) << 0) | ((g) << 2) | ((b) << 4)))
uint32_t pack_pixel(uint8_t pix);
// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
void vga_init(void);               // GPIO + DMA + TIM1 pixel clock setup
void vga_timing_init(void);        // TIM3/TIM4/TIM5 setup
void vga_timing_start(void);       // Enable timers

// Pixel packing
void vga_pack_line(const uint8_t *src);
void vga_pack_line_to(const uint8_t *src, uint32_t *dst);


#endif

