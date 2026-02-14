/*
 * video.h
 *
 *  Created on: Feb 2, 2026
 *      Author: Andy Nicholson
 */

#ifndef INC_VIDEO_H_
#define INC_VIDEO_H_

#include <stdint.h>
#include "vga.h"


// ------------------------------------------------------------
// Externs for buffers and state
// ------------------------------------------------------------
extern uint8_t  linebufA[H_VISIBLE_PIX];
extern uint8_t  linebufB[H_VISIBLE_PIX];

extern uint32_t *render_buf;
extern uint32_t *display_buf;

extern uint32_t vga_pixel_bufferA[H_VISIBLE_PIX];
extern uint32_t vga_pixel_bufferB[H_VISIBLE_PIX];

extern uint32_t *dma_buf;
extern uint32_t *pack_buf;

extern uint32_t border_pixel_buffer[H_VISIBLE_PIX];


extern uint16_t physical_line;

extern uint32_t flat[256];

extern uint32_t t5_cnt;
extern uint32_t activeFstart; // start of active frame (in TIM5 ticks)
extern uint32_t activeFend;   // end of active frame (in TIM5 ticks)
extern uint32_t one_line;     // ticks per line
extern uint32_t margin;
extern uint8_t margin_lines;
extern uint8_t line_wait;
extern uint8_t render_line;
extern uint8_t swap_line;
extern uint8_t repeat;
extern uint8_t dmaStarted;
// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
void sync_to_vsync(void);
void vga_init_border_line(void);
void video_line_step(void);
//void video_compute_vertical_window(void);
void video_recompute_vertical_window(void);
void vga_start_dma(const uint32_t *buf);
void video_update_osd_values(void);
void swapBuffers(void);
void ReArmDMA(void);

#endif /* INC_VIDEO_H_ */
