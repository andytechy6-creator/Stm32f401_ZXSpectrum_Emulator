/*
 * spectrum_core.h
 *
 *  Created on: Feb 1, 2026
 *      Author: Andy Nicholson
 */

#ifndef INC_SPECTRUM_CORE_H_
#define INC_SPECTRUM_CORE_H_


#include <stdint.h>

void spectrum_init(void);
void spectrum_reset(void);
void spectrum_step(void);     // run one CPU tick
uint8_t spectrum_read(uint16_t addr);
void spectrum_write(uint16_t addr, uint8_t value);
void spectrum_port_fe_write(uint8_t value);
uint8_t spectrum_port_fe_read(void);
uint8_t spectrum_in(uint16_t port);
void spectrum_out(uint16_t port, uint8_t value);

extern uint8_t spectrum_ram[0xC000];




#endif /* INC_SPECTRUM_CORE_H_ */
