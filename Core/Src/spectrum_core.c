/*
 * spectrum_core.c
 *
 *  Created on: Feb 1, 2026
 *      Author: Andy Nicholson
 */

#include <string.h>
#include "spectrum_core.h"
#include "spectrum.h"      // for border colour
#include "z80.h"
#include "main.h"

uint8_t spectrum_ram[0xC000];   // 48 KB RAM
extern const uint8_t spectrum_rom_48k[16384];

void spectrum_init(void)
{
	//LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);

    // Load ROM (48K model)
    extern const uint8_t spectrum_rom_48k[16384];
    for (int i = 0; i < 16384; i++)
        spectrum_ram[i] = spectrum_rom_48k[i];

    // Clear RAM
    //for (int i = 0; i < 0xC000; i++)
        //spectrum_ram[i] = 0;
    memset(spectrum_ram, 0, 0xC000);


    spectrum_reset();
}

void spectrum_reset(void)
{
    z80_reset();
}

uint8_t spectrum_read(uint16_t addr)
{
    if (addr < 0x4000)
        return spectrum_rom_48k[addr];

    return spectrum_ram[addr - 0x4000];
}


void spectrum_write(uint16_t addr, uint8_t value)
{
    // ROM is read-only
    if (addr < 0x4000)
        return;

    spectrum_ram[addr - 0x4000] = value;
}

uint8_t spectrum_in(uint16_t port)
{
    // Keyboard + ULA port FE
    if ((port & 0x0001) == 0)
    {
        // TODO: keyboard matrix
        return 0xFF;
    }

    return 0xFF;
}

void spectrum_out(uint16_t port, uint8_t value)
{
    if ((port & 0x0001) == 0)
    {
        // Border colour = bits 0–2
        spectrum_border_colour = value & 0x07;
        return;
    }
}
void spectrum_port_fe_write(uint8_t value)
{
    // Border colour = low 3 bits
    spectrum_border_colour = value & 0x07;

    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);

}
uint8_t spectrum_port_fe_read(void)
{
    // For now: no keys pressed
    return 0xFF;
}
