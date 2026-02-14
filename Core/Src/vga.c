/*
 * vga.c
 *
 *  Created on: Jan 31, 2026
 *      Author: Andy Nicholson
 */

/*
 * vga.c
 * Horizontal timing + DMA + pixel packing
 */

#include "vga.h"
#include "vga_timing.h"
#include "main.h"
#include "video.h"

#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_dma.h"

// External buffer from video.c
extern uint32_t *pack_buf;

// ------------------------------------------------------------
// Pixel packing (8-bit → 32-bit RGB)
// ------------------------------------------------------------
// Pixel format: 0bBB GG RR  (6-bit RGB)
// Hardware pins:
//   PA2 = R0, PA3 = R1
//   PA4 = G0, PA5 = G1
//   PA6 = B0, PA7 = B1
//
// Output word layout (bits 0..5):
//   bit0 = R0
//   bit1 = R1
//   bit2 = G0
//   bit3 = G1
//   bit4 = B0
//   bit5 = B1

inline uint32_t pack_pixel(uint8_t pix)
{
    uint32_t r = (pix >> 0) & 0x03;   // R1:R0
    uint32_t g = (pix >> 2) & 0x03;   // G1:G0
    uint32_t b = (pix >> 4) & 0x03;   // B1:B0

    return (r << 0) | (g << 2) | (b << 4);
}

void vga_pack_line(const uint8_t *src)
{
    for (int i = 0; i < H_VISIBLE_PIX; i++)
        pack_buf[i] = pack_pixel(src[i]) << 2;
}

void vga_pack_line_to(const uint8_t *src, uint32_t *dst)
{
    for (int i = 0; i < H_VISIBLE_PIX; i++)
        dst[i] = pack_pixel(src[i]) << 2;
}


// ------------------------------------------------------------
// VGA GPIO + DMA + TIM1 pixel clock setup
// ------------------------------------------------------------
void vga_init(void)
{

    // Enable GPIOA clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef gpio;
    LL_GPIO_StructInit(&gpio);

    gpio.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 |
               LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    gpio.Mode = LL_GPIO_MODE_OUTPUT;
    gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio.Pull = LL_GPIO_PULL_NO;

    LL_GPIO_Init(GPIOA, &gpio);

    // GPIO already configured by CubeMX (PB0..PB7 or whatever your pins are)
    // DMA2 Stream 5 already configured by CubeMX
    // TIM1 configured for pixel clock by CubeMX

    // Nothing else needed here unless you want to override CubeMX defaults.
    vga_start_dma(display_buf);
}

// ------------------------------------------------------------
// Horizontal + vertical timing (TIM3/TIM4/TIM5)
// ------------------------------------------------------------
void vga_timing_init(void)
{
    // TIM3 = horizontal blanking (OE gating)
    // TIM4 = HSYNC generator
    // TIM5 = VSYNC generator

    // All of these are already configured by CubeMX.
    // This function exists for symmetry and future tweaks.
}

void vga_timing_start(void)
{
    // Start pixel clock
    LL_TIM_EnableCounter(TIM1);

    // Start horizontal blanking
    LL_TIM_EnableCounter(TIM3);

    // Start HSYNC
    LL_TIM_EnableCounter(TIM4);

    // Start VSYNC
    LL_TIM_EnableCounter(TIM5);
}

