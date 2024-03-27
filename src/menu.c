#include "menu.h"

#include "ui.h"
#include "i8080.h"
#include "i8080_hal.h"
#include "reboot.h"
#include "ffs.h"
#include "files.h"
#include "tape.h"
#include "zkg.h"
#include "xprintf.h"
#include "fileman.h"
#include "vga.h"

bool menu_fileman(void)
{
    uint8_t type;
again:
    ui_clear();
    ui_header("RADIO-86RK -->");
    ui_draw_list(
		"1. Programs\n"
		"2. Tapes\n"
	);
    switch (ui_select(2)) {
	case 0:
	    // Программы
	    type = TYPE_ANY;
	    break;
	case 1:
	    // Магнитофонные записи
	    type = TYPE_TAPE;
	    break;
	default:
	    return false;
    }
    // Выбираем файл
again2:
    ui_clear();
    int16_t n = fileman(type, "Select file to load:");
    if (n < 0) goto again;
	if (files_info[n].fattrib & AM_DIR) {
		if (strncmp(files_info[n].name, "..", MAX_WIDTH) == 0) {
			for (int i = strlen(BASE_DIR) - 1; i > 0; --i) {
				if (BASE_DIR[i] == '\\') {
					BASE_DIR[i] = 0;
					files_count = 0;
					ui_clear();
					goto again2;
				}
			}
		}
    	char str[MAX_WIDTH];
    	snprintf(str, MAX_WIDTH, "%s\\%s", BASE_DIR, files_info[n].name);
		stpncpy(BASE_DIR, str, MAX_WIDTH);
		files_count = 0;
		ui_clear();
		goto again2;
	}
    // Загружаем файл
    if (type != TYPE_TAPE) {
		// Загрузка образа в память
		char err[64] = {0};
		int16_t addr = load_file(n, err, 64);
		if (addr >= 0) {
	    	// Нормально загрузилось - запускаем
	    	i8080_jump(addr);
		    // Возвращаемся к эмуляции
		    return true;
		} else {
	    	// Ошибка загрузки файла
	    	ui_clear();
	    	ui_header("RADIO-86RK -->");
	    	ui_draw_text(10, 10, "File load error !");
			ui_draw_text(10, 11, err);
	    	ui_sleep(1000);
	    	goto again2;
		}
    } else {
		// Загрузка с магнитофона
		tape_load(n);
		return true;
    }
}

void menu(void) {
    char str[32];
again:
    ui_clear();
    ui_header("RADIO-86RK -->");
    ui_draw_list(
		"1. (  F11 )  Return to the monitor (no cleanup)\n"
		"2. (  F12 )  File manager\n"
		"3. (Pause )  Full reset\n"
		"4. (PrnSrn)  Change color (palleter)\n"
	);
    ui_draw_text(10, 16,
		"Keyboard mapping:\n"
		"F1-F4   - F1-F4          BK  - Enter\n"
		"AR2     - Alt            PC  - Right Enter\n"
		"RUS/LAT - Caps Lock      3B  - Backspace\n"
		"YC      - CTRL           \\   - Home\n"
		"CC      - Shift          CTP - End/Delete\n"
	"\n"
	"\n"
	"Emulation Management:\n"
	"F5 - F10    - Call ROM E000 + n*4\n"
	"Scroll Lock - Turbo mode ON/OFF\n"
	"WIN+Cursor  - Shift screen\n"
	"MENU        - Help about Radio-86RK\n"
	);
    xsprintf(str, "RK8266 for ZX Mulmulator");
    ui_draw_text(64+6-ets_strlen(str), 33, str);
    switch (ui_select(4))
    {
	case 0:
	    // Возврат в монитор
	    i8080_jump(0xF800);
	    break;
	case 1:
	    // Файловый менеджер
	    if (! menu_fileman()) goto again;
	    break;
	case 2:
	    // Полный сброс
	    ets_memset(i8080_hal_memory(), 0x00, 0x8000);
	    i8080_init();
	    i8080_jump(0xF800);
	    break;
	case 3:
	    change_color();
	    break;
    }
}
