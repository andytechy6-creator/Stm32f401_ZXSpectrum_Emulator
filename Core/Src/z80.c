/*
 * z80.c
 *
 *  Created on: Feb 1, 2026
 *      Author: Andy Nicholson
 */


#include "z80.h"
#include "spectrum_core.h"
#include "main.h"

z80_t z80;

void z80_reset(void)
{
    z80.pc = 0x0000;
    z80.sp = 0xFFFF;
    z80.af = z80.bc = z80.de = z80.hl = 0;
    z80.af2 = z80.bc2 = z80.de2 = z80.hl2 = 0;
    z80.ix = z80.iy = 0;
    z80.iff1 = z80.iff2 = 0;
    z80.im = 0;
}

static inline uint8_t READ(uint16_t a)  { return spectrum_read(a); }
static inline void     WRITE(uint16_t a, uint8_t v) { spectrum_write(a, v); }

void z80_step(void)
{
	LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
    uint8_t op = READ(z80.pc++);

    switch (op)
    {
        case 0x00:  // NOP
            break;

        case 0x3E:  // LD A,n
        {
            z80.af = (z80.af & 0x00FF) | (READ(z80.pc++) << 8);
            break;
        }
        case 0x32:  // LD (nn),A
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            uint16_t addr = lo | (hi << 8);
            WRITE(addr, z80.af >> 8);
            break;
        }
        case 0x18: // JR e
        {
            int8_t e = READ(z80.pc++);
            z80.pc += e;
            break;
        }

        case 0x20: { // JR NZ,e
            int8_t e = READ(z80.pc++);
            if (!(z80.af & 0x01))  // Z flag = bit 0 of F
                z80.pc += e;
            break;
        }


        case 0x28: // JR Z,e
        {
            int8_t e = READ(z80.pc++);
            if (z80.af & 0x01)
                z80.pc += e;
            break;
        }

        case 0x21:  // LD HL,nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            z80.hl = lo | (hi << 8);
            break;
        }

        case 0x11:  // LD DE,nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            z80.de = lo | (hi << 8);
            break;
        }

        case 0x01:  // LD BC,nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            z80.bc = lo | (hi << 8);
            break;
        }

        case 0x77:  // LD (HL),A
        {
            WRITE(z80.hl, z80.af >> 8);
            break;
        }
        case 0x23:  // INC HL
        {
            z80.hl++;
            break;
    	}
        case 0x13:  // INC DE
        {
            z80.de++;
            break;
    	}
        case 0x0B:  // DEC BC
        {
            z80.bc--;
            break;
        }
        case 0xC3:  // JP nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            z80.pc = lo | (hi << 8);
            break;
        }

        case 0xCD:  // CALL nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            uint16_t addr = lo | (hi << 8);

            z80.sp -= 2;
            WRITE(z80.sp, z80.pc & 0xFF);
            WRITE(z80.sp + 1, z80.pc >> 8);

            z80.pc = addr;
            break;
        }

        case 0xC9:  // RET
        {
            uint8_t lo = READ(z80.sp++);
            uint8_t hi = READ(z80.sp++);
            z80.pc = lo | (hi << 8);
            break;
        }
        case 0xD3:
        {   // OUT (n),A
            uint8_t port = READ(z80.pc++);
            if (port == 0xFE)
                spectrum_port_fe_write(z80.af >> 8);
            break;
        }
        case 0xAF: // XOR A
        {
            z80.af = (z80.af & 0x00FF); // A=0, flags mostly zero
            break;
        }
        case 0x3C: // INC A
            {
                uint8_t A = z80.af >> 8;
                A++;
                z80.af = (A << 8) | (z80.af & 0x00FF);
            }
            break;

        case 0x3D: // DEC A
            {
                uint8_t A = z80.af >> 8;
                A--;
                z80.af = (A << 8) | (z80.af & 0x00FF);
            }
            break;
        case 0x7E: // LD A,(HL)
            z80.af = (READ(z80.hl) << 8) | (z80.af & 0x00FF);
            break;

        case 0x36: // LD (HL),n
            WRITE(z80.hl, READ(z80.pc++));
            break;
        case 0x76: // HALT
            // simplest possible: just don't advance PC
            break;
        case 0xFB: z80.iff1 = z80.iff2 = 1; break; // EI
        case 0xF3: z80.iff1 = z80.iff2 = 0; break; // DI
        case 0xFF: // RST 38h
        {
            z80.sp -= 2;
            WRITE(z80.sp, z80.pc & 0xFF);
            WRITE(z80.sp+1, z80.pc >> 8);
            z80.pc = 0x0038;
            break;
        }
        case 0x22: // LD (nn),HL
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            uint16_t addr = lo | (hi << 8);
            WRITE(addr, z80.hl & 0xFF);
            WRITE(addr+1, z80.hl >> 8);
            break;
        }

        case 0x2A: // LD HL,(nn)
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            uint16_t addr = lo | (hi << 8);
            uint8_t l = READ(addr);
            uint8_t h = READ(addr+1);
            z80.hl = l | (h << 8);
            break;
        }
        case 0x10: // DJNZ e
        {
            int8_t e = READ(z80.pc++);
            uint8_t B = z80.bc >> 8;
            B--;
            z80.bc = (B << 8) | (z80.bc & 0x00FF);
            if (B != 0)
                z80.pc += e;
            break;
        }
        case 0xFE: // CP n
        {
            uint8_t A = z80.af >> 8;
            uint8_t n = READ(z80.pc++);
            uint8_t r = A - n;
            // set Z flag if equal
            if (r == 0) z80.af |= 0x01;
            else        z80.af &= ~0x01;
            break;
        }

        case 0xBE: // CP (HL)
        {
            uint8_t A = z80.af >> 8;
            uint8_t n = READ(z80.hl);
            uint8_t r = A - n;
            if (r == 0) z80.af |= 0x01;
            else        z80.af &= ~0x01;
            break;
        }
        case 0xDB: // IN A,(n)
        {
            uint8_t port = READ(z80.pc++);
            if (port == 0xFE)
            {
                z80.af = (spectrum_port_fe_read() << 8) | (z80.af & 0x00FF);
            }
            else
            {
                z80.af = (0xFF << 8) | (z80.af & 0x00FF);
            }
            break;
        }




        case 0x30: // JR NC,e
        {
            int8_t e = READ(z80.pc++);
            if (!(z80.af & 0x01))  // crude: reuse Z as NC for now
                z80.pc += e;
            break;
        }

        case 0x38: // JR C,e
        {
            int8_t e = READ(z80.pc++);
            if (z80.af & 0x01)     // crude: reuse Z as C for now
                z80.pc += e;
            break;
        }






            break;
        case 0xC0: // RET NZ
        	{
					if (!(z80.af & 0x01))
					{
						uint8_t lo = READ(z80.sp++);
						uint8_t hi = READ(z80.sp++);
						z80.pc = lo | (hi << 8);
					}
            break;
        	}
        case 0xC8:
           {// RET Z
             if (z80.af & 0x01)
             {
                uint8_t lo = READ(z80.sp++);
                uint8_t hi = READ(z80.sp++);
                z80.pc = lo | (hi << 8);
             }
            break;
           }
        case 0xC4: // CALL NZ,nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            uint16_t addr = lo | (hi << 8);
            if (!(z80.af & 0x01))
            {
                z80.sp -= 2;
                WRITE(z80.sp, z80.pc & 0xFF);
                WRITE(z80.sp+1, z80.pc >> 8);
                z80.pc = addr;
            }
            break;
        }

        case 0xCC: // CALL Z,nn
        {
            uint16_t lo = READ(z80.pc++);
            uint16_t hi = READ(z80.pc++);
            uint16_t addr = lo | (hi << 8);
            if (z80.af & 0x01)
            {
                z80.sp -= 2;
                WRITE(z80.sp, z80.pc & 0xFF);
                WRITE(z80.sp+1, z80.pc >> 8);
                z80.pc = addr;
            }
            break;
        }


        default:
            // For now, unknown opcodes just NOP
            break;
    }
}
