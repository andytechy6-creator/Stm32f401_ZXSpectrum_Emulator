/*
 * video.c
 *
 *  Created on: Feb 2, 2026
 *      Author: Andy Nicholson
 */

/*
 * video.c
 * Vertical timing + line mapping + ping-pong buffering
 */

#include "video.h"
#include "vga.h"// for vga_pack_line(), vga_pack_line_to()
#include "main.h"
#include <stdio.h>
#include "spectrum.h"          // for spectrum_render_scanline()
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_dma.h"

uint16_t new_line_ready;
// font table
// 6x8 font, ASCII 32–95
static const uint8_t font6x8[64][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // 32  ' '
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // 33  '!'
    {0x00,0x07,0x00,0x07,0x00,0x00}, // 34  '"'
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // 35  '#'
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // 36  '$'
    {0x23,0x13,0x08,0x64,0x62,0x00}, // 37  '%'
    {0x36,0x49,0x55,0x22,0x50,0x00}, // 38  '&'
    {0x00,0x05,0x03,0x00,0x00,0x00}, // 39  '''
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // 40  '('
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // 41  ')'
    {0x14,0x08,0x3E,0x08,0x14,0x00}, // 42  '*'
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // 43  '+'
    {0x00,0x50,0x30,0x00,0x00,0x00}, // 44  ','
    {0x08,0x08,0x08,0x08,0x08,0x00}, // 45  '-'
    {0x00,0x60,0x60,0x00,0x00,0x00}, // 46  '.'
    {0x20,0x10,0x08,0x04,0x02,0x00}, // 47  '/'
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 48  '0'
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // 49  '1'
    {0x42,0x61,0x51,0x49,0x46,0x00}, // 50  '2'
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // 51  '3'
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // 52  '4'
    {0x27,0x45,0x45,0x45,0x39,0x00}, // 53  '5'
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 54  '6'
    {0x01,0x71,0x09,0x05,0x03,0x00}, // 55  '7'
    {0x36,0x49,0x49,0x49,0x36,0x00}, // 56  '8'
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // 57  '9'
    {0x00,0x36,0x36,0x00,0x00,0x00}, // 58  ':'
    {0x00,0x56,0x36,0x00,0x00,0x00}, // 59  ';'
    {0x08,0x14,0x22,0x41,0x00,0x00}, // 60  '<'
    {0x14,0x14,0x14,0x14,0x14,0x00}, // 61  '='
    {0x00,0x41,0x22,0x14,0x08,0x00}, // 62  '>'
    {0x02,0x01,0x51,0x09,0x06,0x00}, // 63  '?'
    {0x32,0x49,0x79,0x41,0x3E,0x00}, // 64  '@'
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, // 65  'A'
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // 66  'B'
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // 67  'C'
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // 68  'D'
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // 69  'E'
    {0x7F,0x09,0x09,0x09,0x01,0x00}, // 70  'F'
    {0x3E,0x41,0x49,0x49,0x7A,0x00}, // 71  'G'
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // 72  'H'
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // 73  'I'
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // 74  'J'
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // 75  'K'
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // 76  'L'
    {0x7F,0x02,0x0C,0x02,0x7F,0x00}, // 77  'M'
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // 78  'N'
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // 79  'O'
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // 80  'P'
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // 81  'Q'
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // 82  'R'
    {0x46,0x49,0x49,0x49,0x31,0x00}, // 83  'S'
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // 84  'T'
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // 85  'U'
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // 86  'V'
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, // 87  'W'
    {0x63,0x14,0x08,0x14,0x63,0x00}, // 88  'X'
    {0x07,0x08,0x70,0x08,0x07,0x00}, // 89  'Y'
    {0x61,0x51,0x49,0x45,0x43,0x00}, // 90  'Z'
    {0x00,0x7F,0x41,0x41,0x00,0x00}, // 91  '['
    {0x02,0x04,0x08,0x10,0x20,0x00}, // 92  '\'
    {0x00,0x41,0x41,0x7F,0x00,0x00}, // 93  ']'
    {0x04,0x02,0x01,0x02,0x04,0x00}, // 94  '^'
    {0x40,0x40,0x40,0x40,0x40,0x00}, // 95  '_'
};

void draw_debug_osd_line(uint16_t y, uint32_t *buf);
void video_update_osd_values(void);

extern const int step_table[];
extern uint8_t step_index;
extern uint8_t sign_positive;


#define BLUE   0b000011
#define RED    0b110000
#define GREEN  0b001100
#define CYAN   0b001111
#define OSD_WHITE   (pack_pixel(0x3F) << 2)   // bright white in your format

// ------------------------------------------------------------
// Buffers + state
// ------------------------------------------------------------
uint32_t linebufa[H_VISIBLE_PIX];
uint32_t linebufb[H_VISIBLE_PIX];

uint32_t *render_buf  = linebufa;
uint32_t *display_buf = linebufb;



// This is the actual storage
//uint32_t flat[256];


uint32_t vga_pixel_bufferA[H_VISIBLE_PIX] __attribute__((aligned(4)));
uint32_t vga_pixel_bufferB[H_VISIBLE_PIX] __attribute__((aligned(4)));
uint32_t border_pixel_buffer[H_VISIBLE_PIX] __attribute__((aligned(4)));


uint32_t t5_cnt = 0;
uint32_t activeFstart = 0;
uint32_t activeFend = 0;
uint32_t one_line = 0;
uint32_t margin = 0;
uint8_t  margin_lines = 2;
uint8_t  line_wait = 20;

uint8_t dmaStarted;
//uint32_t *dma_buf  = vga_pixel_bufferA;  // DMA reads from here
//uint32_t *pack_buf = vga_pixel_bufferB;  // CPU packs into here
uint16_t phys_visible_start;
uint16_t phys_visible_end;

uint16_t active_line = 0;

uint8_t swap_line  = 1;
uint8_t render_line = 0;
// for draw_debug_osd_line
static char debug1[32];
static char debug2[32];
static char debug3[32];
static char debug4[32];
static char debug5[32];
static char debug6[32];
static char debug7[32];
static char debug8[32];
static char debug9[32];
// ------------------------------------------------------------
// Sync to VSYNC (TIM2 update)
// ------------------------------------------------------------
void sync_to_vsync(void)
{
    while (!LL_TIM_IsActiveFlag_UPDATE(TIM2)) { }
    LL_TIM_ClearFlag_UPDATE(TIM2);
}

// ------------------------------------------------------------
// Start DMA for one scanline
// ------------------------------------------------------------
void vga_start_dma(const uint32_t *buf)
{
    // 1. Disable stream
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);

    // 2. Wait until EN bit is actually cleared
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {
        // spin
    }

    // 3. Clear all pending flags for this stream
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_TE5(DMA2);
    LL_DMA_ClearFlag_DME5(DMA2);
    LL_DMA_ClearFlag_FE5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);

    // 4. Configure addresses and length
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)&GPIOA->ODR);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)buf);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, H_VISIBLE_PIX);

    // 5. Enable stream
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
}

void ReArmDMA(void)
	{
		LL_DMA_ClearFlag_TC5(DMA2);
		LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)display_buf);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
	}
// ------------------------------------------------------------
// Pre-pack a border line (black)
// ------------------------------------------------------------
void vga_init_border_line(void)
{
    static uint8_t tmp[H_VISIBLE_PIX];

    for (int i = 0; i < H_VISIBLE_PIX; i++)
        tmp[i] = 0x3F;   // bright white (R=3,G=3,B=3)

    vga_pack_line_to(tmp, border_pixel_buffer);
}


void render_line_into(uint32_t *buf, int line)
{
    uint8_t pix;

    int band = (line / 40) % 3;


    if (band == 0)      pix = 0b000011;   // red
    else if (band == 1) pix = 0b001100;   // green
    else                pix = 0b110000;   // blue

    uint32_t packed = pack_pixel(pix) << 2;

    for (int i = 0; i < H_VISIBLE_PIX; i++)
        buf[i] = packed;
}

static void fill_solid(uint32_t *buf, uint8_t pix)
{
    uint32_t packed = pack_pixel(pix) << 2;

    for (int i = 0; i < H_VISIBLE_PIX; i++)
        buf[i] = packed;
}


void fill_border(uint32_t *buf)
{
    // simple black border
    for (int i = 0; i < H_VISIBLE_PIX; i++)
        buf[i] = 0;   // all colour bits = 0
}

void video_recompute_vertical_window(void)
{
	getTimes();
    one_line = TIM4->ARR + 1;

    activeFstart = TIM5->CCR1 / one_line;
    activeFend   = TIM5->ARR  / one_line;

    // safety margin: 1–2 lines before active
    //margin = 2;

    // any other derived values you need
}

#if 0
void video_compute_vertical_window(void)
{
    uint32_t t5_ccr1 = TIM5->CCR1;
    uint32_t t5_arr  = TIM5->ARR;
    uint32_t t4_arr  = TIM4->ARR;

    float ticks_per_line = (float)(t4_arr + 1);

    float start_f = (float)t5_ccr1 / ticks_per_line;
    float end_f   = (float)t5_arr  / ticks_per_line;

}
#endif
#if 0
void video_compute_vertical_window(void)
{
    uint32_t t5_ccr1 = TIM5->CCR1;
    uint32_t t5_arr  = TIM5->ARR;
    uint32_t t4_arr  = TIM4->ARR;

    float ticks_per_line = (float)(t4_arr + 1);

    float start_f = (float)t5_ccr1 / ticks_per_line;
    float end_f   = (float)t5_arr  / ticks_per_line;

    phys_visible_start = (uint16_t)(start_f + 0.5f);
    phys_visible_end   = (uint16_t)(end_f   + 0.5f);


    int max_up_shift = phys_visible_start;  // can't go above 0
    int shift = 20;                         // your desired shift

    if (shift > max_up_shift)
        shift = max_up_shift;

    phys_visible_start += shift;
    phys_visible_end   += shift;


}
#endif
#if 0
void video_compute_vertical_window(void)
{
    phys_visible_start = 0;     // or whatever your real first visible line is
    phys_visible_end   = 240;   // visible height
}

#endif
// ------------------------------------------------------------
// Per-HSYNC line processing
// ------------------------------------------------------------
#if 1 // third and should be correct fix is also black  and usb stops working
uint8_t repeat = 0;

void video_line_step(void)
{
    // 1. Latch VSYNC BEFORE waiting for HSYNC
    uint8_t vsync_now = (GPIOA->IDR & V_BLANK_Pin);

    // 2. Wait for HSYNC (DMA for this line will start right after)
    //while (!LL_TIM_IsActiveFlag_UPDATE(TIM4)) {}
    //while (TIM4->CNT <  TIM4->CCR1 - 20 && TIM4->CNT >0)
    while (GPIOB->IDR & H_BLANK_Pin) {}

    // 3. Update line counter
    if (vsync_now) {
        active_line = 0;
    } else {
        active_line++;
    }

    dmaStarted = 1;
    // 4. If this is the FIRST send, start rendering the next line immediately
    if (repeat == render_line) {

        // 1. Render background into render_buf
        if (active_line < 60) {
            fill_solid(render_buf, 0b110000);
        } else if (active_line < 120) {
            fill_solid(render_buf, 0b000011);
        } else if (active_line < 180) {
            fill_solid(render_buf, 0b001100);
        } else if (active_line < 240) {
            fill_solid(render_buf, 0b111100);
        } else {
            fill_border(render_buf);
        }

        // 2. Draw OSD on top
        draw_debug_osd_line(active_line, render_buf);
    }
    //dmaStarted = 1;


#if 1
    // 5. Wait for DMA complete (line fully sent)
    while (!LL_DMA_IsActiveFlag_TC5(DMA2)) {}
    LL_DMA_ClearFlag_TC5(DMA2);

    // 6. If this was the SECOND send, NOW it is safe to swap


    if (repeat == swap_line)
    {

    	swapBuffers();
        //uint32_t *tmp = display_buf;
        //display_buf   = render_buf;
        //render_buf    = tmp;
    }

    // 7. Re-arm DMA for next HSYNC with the (possibly new) display_buf
    ReArmDMA();

    // 8. Toggle repeat
    repeat ^= 1;

#endif

}
#endif


static inline void put_pixel(uint32_t *buf, int x, uint32_t color)
{
    if (x >= 0 && x < H_VISIBLE_PIX)
        buf[x] = color;
}

void draw_char_line(uint32_t *buf, int x, int glyph_row, char c, uint32_t color)
{
    if (c < 32 || c > 95)
        return;

    const uint8_t *glyph = font6x8[c - 32];

    for (int col = 0; col < 6; col++)
    {
        uint8_t bits = glyph[col];

        // Only draw the bit for THIS scanline
        if (bits & (1 << glyph_row))
            put_pixel(buf, x + col, color);
    }
}
#if 0
void draw_char(uint32_t *buf, int x, int y, char c, uint32_t color)
{
    if (c < 32 || c > 95)
        return;

    const uint8_t *glyph = font6x8[c - 32];

    for (int col = 0; col < 6; col++)
    {
        uint8_t bits = glyph[col];

        for (int row = 0; row < 8; row++)
        {
            if (bits & (1 << row))
                put_pixel(buf, x + col, color);
        }
    }
}
#endif

void draw_text_line(uint32_t *buf, int x, int glyph_row, const char *s, uint32_t color)
{
    while (*s)
    {
        draw_char_line(buf, x, glyph_row, *s, color);
        x += 6;
        s++;
    }
}
#if 1
void swapBuffers(void)
{
    if (repeat == swap_line) {
        uint32_t *tmp = display_buf;
        display_buf   = render_buf;
        render_buf    = tmp;
    }
}
#endif


void draw_debug_osd_line(uint16_t active_line, uint32_t *buf)
{
    // OSD occupies lines 0–39 (5 rows × 8 pixels)
	//int vis = active_line - phys_visible_start;
	//int vis = active_line;


    if (active_line >= 0)
    {
        int glyph_row = active_line % 8;
        int text_row  = active_line / 8;

        switch (text_row)
        {
            case 0: draw_text_line(buf, 10, glyph_row, debug1, OSD_WHITE); break;
            case 1: draw_text_line(buf, 10, glyph_row, debug2, OSD_WHITE); break;
            case 2: draw_text_line(buf, 10, glyph_row, debug3, OSD_WHITE); break;
            case 3: draw_text_line(buf, 10, glyph_row, debug4, OSD_WHITE); break;
            case 4: draw_text_line(buf, 10, glyph_row, debug5, OSD_WHITE); break;
            case 5: draw_text_line(buf, 10, glyph_row, debug6, OSD_WHITE); break;
            case 6: draw_text_line(buf, 10, glyph_row, debug7, OSD_WHITE); break;
            case 7: draw_text_line(buf, 10, glyph_row, debug8, OSD_WHITE); break;
            case 8: draw_text_line(buf, 10, glyph_row, debug9, OSD_WHITE); break;
            //case 6: draw_text_line(buf, 10, glyph_row, debug7, OSD_WHITE); break;
        }
    }
}


#if 0
void draw_debug_osd_line(uint16_t active_line, uint32_t *buf)
{
    // Only draw OSD in the top 40 lines
    if (active_line >= 0 && active_line < 8)
    	draw_text(buf, 10, 0, debug1, OSD_WHITE);

    else if (active_line >= 8 && active_line < 16)
    	draw_text(buf, 10, 0, debug1, OSD_WHITE);

    else if (active_line >= 16 && active_line < 24)
    	draw_text(buf, 10, 0, debug1, OSD_WHITE);

    else if (active_line >= 24 && active_line < 32)
    	draw_text(buf, 10, 0, debug1, OSD_WHITE);

    else if (active_line >= 32 && active_line < 40)
    	draw_text(buf, 10, 0, debug1, OSD_WHITE);
}
#endif
void video_update_osd_values(void)
{
    sprintf(debug1, "T3 ARR: %lu", TIM3->ARR);
    sprintf(debug2, "T3 CCR: %lu", TIM3->CCR4);
    sprintf(debug3, "T5 ARR: %lu", TIM5->ARR);
    sprintf(debug4, "T5 CCR: %lu", TIM5->CCR1);
    sprintf(debug5, "STEP:%d SIGN:%c", step_table[step_index], sign_positive ? '+' : '-');
    sprintf(debug6, "LINE WAIT: %u", line_wait);
    sprintf(debug7, "RENDER_LINE: %u", render_line);
    sprintf(debug8, "SWAP_LINE: %u", swap_line);
    sprintf(debug9, "MARGIN: %lu", margin);
}

const int step_table[] = {0,1,2,3,4,5,6,7}; uint8_t step_index = 0; uint8_t sign_positive = 1;
