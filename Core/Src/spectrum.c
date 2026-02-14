/*
 * spectrum.c
 *
 *  Created on: Jan 31, 2026
 *      Author: Andy Nicholson
 */

#include "spectrum_core.h"

#include "spectrum.h"
#include "vga.h"

// These will be defined/assigned elsewhere in your emulator core
//uint8_t *spectrum_ram = 0;          // must point to 48K RAM (or more)
uint8_t  spectrum_border_colour = 0; // default border = black

// Simple mapping from Spectrum colour index (0–7) to your 2:2:2 RGB byte
// You can refine this later to match real Spectrum palette.
static uint8_t spectrum_colour_lut[8] =
{
    0x00, // 0: black
    0x03, // 1: blue
    0x0C, // 2: red
    0x0F, // 3: magenta
    0x30, // 4: green
    0x33, // 5: cyan
    0x3C, // 6: yellow
    0x3F  // 7: white
};

// Horizontal crop: we only have 210 pixels, Spectrum is 256 wide.
// This offset decides which 210 pixels we show.
// For now, centre it: (256 - 210) / 2 = 23
#define SPEC_CROP_OFFSET  23

void spectrum_render_scanline(uint16_t y, uint8_t *dst)
{
    // y is logical line within the VGA frame (0..MON_VISIBLE_LINES-1)
    // First, map to Spectrum bitmap space (0..191), or border.
	if (y < SPEC_Y_START || y >= SPEC_Y_END)

    {
        // Entire line is border
        uint8_t border_pix = spectrum_colour_lut[spectrum_border_colour & 0x07];
        for (int x = 0; x < H_VISIBLE_PIX; x++)
            dst[x] = border_pix;
        return;
    }

    uint16_t sy = y - SPEC_Y_START; // 0..191

    // Compute Spectrum bitmap row address using real 3-block layout.
    // Base bitmap at 0x4000 in Spectrum RAM.
    uint16_t row_addr =
          ((sy & 0x07) << 8)   // bits 0-2: row within 8-line block
        | ((sy & 0x38) << 2)   // bits 3-5: which 8-line block
        | ((sy & 0xC0) << 5);  // bits 6-7: which third of screen

    uint8_t *bitmap_row = spectrum_ram + 0x4000 + row_addr;
    uint8_t *attr_base  = spectrum_ram + 0x5800;

    // Render 210 pixels with horizontal crop
    for (int x = 0; x < H_VISIBLE_PIX; x++)
    {
        int sx = x + SPEC_CROP_OFFSET; // 0..255 (or outside)

        if (sx < 0 || sx >= 256)
        {
            // Outside Spectrum area horizontally: border
            dst[x] = spectrum_colour_lut[spectrum_border_colour & 0x07];
            continue;
        }

        // Which 8x8 attribute cell?
        uint8_t attr_col = sx >> 3;      // 0..31
        uint8_t attr_row = sy >> 3;      // 0..23
        uint16_t attr_index = attr_row * 32 + attr_col;
        uint8_t attr = attr_base[attr_index];

        uint8_t ink   = attr & 0x07;
        uint8_t paper = (attr >> 3) & 0x07;
        uint8_t bright = (attr & 0x40) ? 1 : 0;
        uint8_t flash  = (attr & 0x80) ? 1 : 0;

        // TODO: later: implement flash using a global frame counter.
        (void)flash;

        // Fetch bitmap bit
        uint8_t x_byte = sx >> 3;        // 0..31
        uint8_t bit    = 7 - (sx & 7);   // MSB first
        uint8_t pixbyte = bitmap_row[x_byte];

        uint8_t is_ink = (pixbyte >> bit) & 1;

        uint8_t colour_index = is_ink ? ink : paper;

        // Simple bright handling: map to higher intensity entries in LUT later if you like.
        // For now, ignore bright or fold it into LUT if you want.
        (void)bright;

        dst[x] = spectrum_colour_lut[colour_index & 0x07];
    }
}
