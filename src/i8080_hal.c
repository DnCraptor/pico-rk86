#include "i8080_hal.h"

#include <hardware/pwm.h>

#include "vg75.h"
#include "zkg.h"
#include "vv55_i.h"
#include "rom.h"
#include "mikrosha_rom.h"
#include "align4.h"
#include "board.h"

// 0x0000..0x7FFF
uint8_t RAM[0x8000];
// 0x8000..0x9FFF // ВВ55 внутренняя

// 0xA000..0xBFFF
uint8_t RAM2[0x2000];
// 0xC000..0xDFFF // ВГ75 + шрифты

// 0xE000..0xF800
uint8_t ROM[0x1800];

uint32_t i8080_cycles = 0;

int i8080_hal_memory_read_word(int addr)
{
    return 
        (i8080_hal_memory_read_byte(addr + 1) << 8) |
        i8080_hal_memory_read_byte(addr);
}

void i8080_hal_memory_write_word(int addr, int word) {
    i8080_hal_memory_write_byte(addr, word & 0xff);
    i8080_hal_memory_write_byte(addr + 1, (word >> 8) & 0xff);
}

uint8_t i8080_hal_memory_read_byte(int addr) {
	//printf("i8080_hal_memory_read_byte %ph", addr);
    if ( (addr & 0x8000) == 0 ) {
		// ОЗУ
		//	printf("i8080_hal_memory_read_byte RAM[%04Xh] %02Xh", addr & 0x7fff, RAM[addr & 0x7fff]);
		return RAM[addr & 0x7fff];
    } else {
		// Переферия/ПЗУ
		switch ((addr >> 12) & 0x0f) {
#if MODEL==MICROSHA
	    case 0x8:
	    case 0xB:
			// extRom
			return 0;
	    case 0xC: // C000-C7FF ? + ppi2 C800-CFFF
			// ВВ55 внутренняя
			return vv55_i_R(addr & 0x03);
	    case 0xD:
			// ВГ75 + шрифты
			if (addr & (1 << 10))	// A10 - переключатель ВГ75/шрифт
			{
		    	// Шрифт (A12,A11 - номер шрифта)
		    	uint8_t n = (addr >> 11) & 0x03;
		    	addr &= 0x3FF;
		    	addr = ((addr & 0x07) << 7) | (addr >> 3);	// меняем адресацию
		    	return zkg[n][addr] ^ 0xFF;
			}
		    // ВГ75
		    return vg75_R(addr & 1);
	    case 0xE:
    		// ПЗУ? вместо ИК57 (ИК57 никто не читает)
	    	return ROM[addr & 0x1FFF];
	    case 0xF:
			if (addr < 0x8F00) {
				//printf("i8080_hal_memory_read_byte ROM[%04Xh] %02Xh", addr & 0x1FFF, ROM[addr & 0x1FFF]);
				return ROM[addr & 0x1FFF];
			}
    		return mikrosha_rom[addr - 0xF800];
#else
	    case 0x8:
	    case 0x9:
			// ВВ55 внутренняя
			return vv55_i_R(addr & 0x03);
	    case 0xA:
	    case 0xB:
			// Доп.ОЗУ вместо ВВ55
			return RAM2[addr & 0x1FFF];
	    case 0xC:
	    case 0xD:
			// ВГ75 + шрифты
			if (addr & (1 << 10))	// A10 - переключатель ВГ75/шрифт
			{
		    	// Шрифт (A12,A11 - номер шрифта)
		    	uint8_t n = (addr >> 11) & 0x03;
		    	addr &= 0x3FF;
		    	addr = ((addr & 0x07) << 7) | (addr >> 3);	// меняем адресацию
		    	return zkg[n][addr] ^ 0xFF;
			}
		    // ВГ75
		    return vg75_R(addr & 1);
	    case 0xE:
    		// ПЗУ? вместо ИК57 (ИК57 никто не читает)
	    	return ROM[addr & 0x1FFF];
	    case 0xF:
			if (addr < 0x8F00) {
				//printf("i8080_hal_memory_read_byte ROM[%04Xh] %02Xh", addr & 0x1FFF, ROM[addr & 0x1FFF]);
				return ROM[addr & 0x1FFF];
			}
			return ROM_F800[addr - 0xF800];
#endif
		    default:
				return 0x00;
		}
    }
}

void i8080_hal_memory_write_byte(int addr, int byte) {
    if ( (addr & 0x8000) == 0 ) {
		// ОЗУ
		RAM[addr & 0x7fff] = byte;
    } else {
		// Переферия
		switch ((addr >> 12) & 0x0f) {
	    	case 0xC: // 2x?
				// ВВ55 внутренняя
				vv55_i_W(addr & 0x03, byte);
				break;
			case 0xD:
				if (addr >= 0xD800) {
					// TODO: 8253
					break;
				}
	    	case 0xE:
				// ВГ75 + шрифты
				if (addr & (1 << 10))	// A10 - переключатель ВГ75/шрифт
				{
				 	// Шрифт (A12,A11 - номер шрифта)
		    		uint8_t n = (addr >> 11) & 0x03;
		    		addr &= 0x3FF;
		    		addr = ((addr & 0x07) << 7) | (addr >> 3);	// меняем адресацию
			    	if (n!=0) zkg[n][addr] = byte ^ 0xFF;	// 0-й шрифт запрещаем менять
				} else	{
			    	// ВГ75
		    		vg75_W(addr & 1, byte);
				}
				break;
#if MODEL==MICROSHA
#else
	    	case 0x8:
	    	case 0x9:
				// ВВ55 внутренняя
				vv55_i_W(addr & 0x03, byte);
				break;
	    	case 0xA:
	    	case 0xB:
				// Доп.ОЗУ вместо ВВ55
				RAM2[addr & 0x1FFF]=byte;
				break;
	    	case 0xC:
	    	case 0xD:
				// ВГ75 + шрифты
				if (addr & (1 << 10))	// A10 - переключатель ВГ75/шрифт
				{
				 	// Шрифт (A12,A11 - номер шрифта)
		    		uint8_t n = (addr >> 11) & 0x03;
		    		addr &= 0x3FF;
		    		addr = ((addr & 0x07) << 7) | (addr >> 3);	// меняем адресацию
			    	if (n!=0) zkg[n][addr] = byte ^ 0xFF;	// 0-й шрифт запрещаем менять
				} else	{
			    	// ВГ75
		    		vg75_W(addr & 1, byte);
				}
				break;
	    	case 0xE:
				// ИК57
				ik57_W(addr & 0x0f, byte);
				break;
#endif
	    	case 0xF:
			default:
				// ПЗУ - записывать нельзя
				break;
		}
    }
}

int i8080_hal_io_input(int port) {
    return 0; // TODO: ??
}

void i8080_hal_io_output(int port, int value) {
	printf("i8080_hal_io_output ignored");
}

void i8080_hal_iff(int on) {
    if (on) pwm_set_gpio_level(BEEPER_PIN, 255);
	else pwm_set_gpio_level(BEEPER_PIN, 0);
}

unsigned char* i8080_hal_memory(void) {
    return RAM;
}

unsigned char* i8080_hal_rom(void) {
    return ROM;
}

void i8080_hal_init(void) {
    // Инитим ОЗУ
    ets_memset(RAM, 0x00, sizeof(RAM));   // 0x0000..0x7FFF
    ets_memset(RAM2, 0x00, sizeof(RAM2)); // 0xA000..0xBFFF
    
    // Инитим ПЗУ
    ets_memset(ROM, 0xFF, sizeof(ROM)); // 0xE000..0xF800
  ///  ets_memcpy(ROM + 0x1800, ROM_F800, sizeof(ROM_F800)); // 0xF800..0xFFFF
}
