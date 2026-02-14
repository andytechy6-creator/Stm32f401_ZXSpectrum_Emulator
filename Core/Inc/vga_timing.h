/*
 * vga_timings.h
 *
 *  Created on: Feb 2, 2026
 *      Author: Andy Nicholson
 */

#ifndef INC_VGA_TIMING_H_
#define INC_VGA_TIMING_H_

#ifndef VGA_TIMING_H
#define VGA_TIMING_H

#include <stdint.h>

// ------------------------------------------------------------
// Pixel clock domain
// ------------------------------------------------------------
#define VGA_PIXEL_CLOCK_HZ    8400000
#define SYSCLK_HZ             84000000
#define TICKS_PER_PIXEL       (SYSCLK_HZ / VGA_PIXEL_CLOCK_HZ)   // = 10

// ------------------------------------------------------------
// Horizontal timing (your output pipeline)
// ------------------------------------------------------------
#if 0
#define H_VISIBLE_PIX         210
#define H_FP_PIX              8
#define H_SYNC_PIX            20
#define H_BP_PIX              28
#define H_TOTAL_PIX  (H_VISIBLE_PIX + H_FP_PIX + H_SYNC_PIX + H_BP_PIX)
#endif

#if 0
// try shifting image right a bit:
#define H_VISIBLE_PIX 210
#define H_FP_PIX      16   // +8
#define H_SYNC_PIX    20
#define H_BP_PIX      20   // -8  (still 266 total)
#endif

#define H_VISIBLE_PIX 256
#define H_FP_PIX      10
#define H_SYNC_PIX    20
#define H_BP_PIX      24   // still 266 total


// ------------------------------------------------------------
// Vertical timing — three coordinate spaces
// ------------------------------------------------------------

// 1) Physical monitor space (real HSYNC count)
#define PHYS_TOTAL_LINES      525

// 2) Output space (your doubled-line output)
#define OUT_VISIBLE_LINES     240
#define OUT_TOTAL_LINES       (OUT_VISIBLE_LINES)
#define OUT_TOTAL_PHYS_LINES  (OUT_TOTAL_LINES * 2)

// 3) Spectrum logical space
#define SPEC_VISIBLE_LINES    192

// ------------------------------------------------------------
// Vertical placement inside the physical frame
// ------------------------------------------------------------

// now calculated from timers

//#define PHYS_VISIBLE_START    28
//#define PHYS_VISIBLE_END      508 //(PHYS_VISIBLE_START + OUT_TOTAL_PHYS_LINES)

// Spectrum centering inside output space
#define OUT_TOP_BORDER        ((OUT_VISIBLE_LINES - SPEC_VISIBLE_LINES) / 2)

// ------------------------------------------------------------
// Mapping helpers




#define VIS_TO_OUT(vis) \
    ((vis) >> 1)

#define OUT_TO_SPEC(out) \
    ((out) - OUT_TOP_BORDER)

#endif


#endif /* INC_VGA_TIMING_H_ */
