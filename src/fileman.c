#include "fileman.h"

#include "ui.h"

#include "ets.h"
#include "ps2_rk.h"
#include "ps2_codes_rk.h"
#include "xprintf.h"
#include "xlat.h"
#include "ffs.h"
#include "vg75.h"

file_info_t files_info[MAX_FILES] = { 0 };
size_t files_count = 0;

static void del(const char* name) {
    char str[MAX_WIDTH];
    xsprintf(str, "Confirm file removement: %s", name);
    if (ui_yes_no(str) == 1) {
		snprintf(str, MAX_WIDTH, "%s\\%s", BASE_DIR, name);
		f_unlink(str);
    }
}

static void rename(const char* old_name) {
    const char * new_name;
again:
    new_name = ui_input_text("Enter new filename:", old_name, MAX_WIDTH >> 1);
    if ( (! new_name) || (! new_name[0]) ) return;
    // Ищем - вдруг такой файл уже есть
    char str[MAX_WIDTH];
	snprintf(str, MAX_WIDTH, "%s\\%s", BASE_DIR, new_name);
    if (f_stat(str, 0) == FR_OK) {
        // Уже есть такой файл
        ui_draw_text(10, 12, "Such name already exists !");
        ui_sleep(2000);
        goto again;
    }
    char o_str[MAX_WIDTH];
	char n_str[MAX_WIDTH];
	snprintf(o_str, MAX_WIDTH, "%s\\%s", BASE_DIR, old_name);
	snprintf(n_str, MAX_WIDTH, "%s\\%s", BASE_DIR, new_name);
    // Переименовываем
    if (f_rename(o_str, n_str) != FR_OK) {
        // Ошибка
        ui_draw_text(10, 12, "Unable to rename file !");
        ui_sleep(2000);
    }
}

static int m_comp(const file_info_t * e1, const file_info_t * e2) {
    if ((e1->fattrib & AM_DIR) && !(e2->fattrib & AM_DIR)) return -1;
    if (!(e1->fattrib & AM_DIR) && (e2->fattrib & AM_DIR)) return 1;
    return strncmp(e1->name, e2->name, MAX_WIDTH >> 1);
}

inline static void m_add_file(FILINFO* fi) {
    if (files_count >= MAX_FILES) {
        // WARN?
        return;
    }
    file_info_t* fp = &files_info[files_count++];
    fp->fattrib = fi->fattrib;
    fp->fdate   = fi->fdate;
    fp->ftime   = fi->ftime;
    fp->fsize   = fi->fsize;
    strncpy(fp->name, fi->fname, MAX_WIDTH >> 1);
}

#define COLS 4
#define FILES 16

int16_t fileman(uint8_t type, const char *text)
{
    DIR dir;
    static int page = 0, n = 0, prev = 0;
    page = 0, n = 0;
reread:
    files_count = 0;
    // Собираем каталог файлов
	if (f_opendir(&dir, BASE_DIR) != FR_OK)
        return -1;
    FILINFO fileInfo;
    while(f_readdir(&dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
        m_add_file(&fileInfo);
    }
    f_closedir(&dir);
    if (strlen(BASE_DIR) > 5) {
        file_info_t* fp = &files_info[files_count++];
        fp->fattrib = AM_DIR;
        fp->fdate   = 0;
        fp->ftime   = 0;
        fp->fsize   = 0;
        strncpy(fp->name, "..", MAX_WIDTH >> 1);
    }
    // Сортируем каталог по имени
	qsort (files_info, files_count, sizeof(file_info_t), m_comp);
    // Рисуем
    ui_clear();
    
    #if MODEL==MICROSHA
        ui_header("MICROSHA -->");
    #else
        ui_header("RADIO-86RK -->");
    #endif
    if (files_count == 0) {
		ui_draw_text(10, 10, "No files !");
		ui_sleep(1000);
		return -1;
    }
    ui_draw_text(10, 9, text);
    ui_draw_text(10, 28, "ENTER - select    SPACE  - rename");
    ui_draw_text(10, 29, "ESC   - cancel    DELETE - remove");
    // Для красивых рамок
    screen.underline_y = 4;
#define PX	8
#define PY	10
    // Горизонтальные линии
 //   for (i = 1; i < 60; i++) {
//		ui_scr[PY +  0][PX + i] = 0xE0;
//		ui_scr[PY + 21][PX + i] = 0xE0;
//    }
    // Вертикальные линии + аттрибуты
//    for (i = 0; i < 20; i++) {
//		for (j = 0; j < 4; j++) {
//	    	ui_scr[PY + 1 + i][PX + j *17 + 0] = 0xE4;
//	    	ui_scr[PY + 1 + i][PX + j *17 + 1] = 0x80;
//	    	ui_scr[PY + 1 + i][PX + j *17 +16] = 0x80;
//		}
//		ui_scr[PY + 1 + i][PX + 4 * 17] = 0xE4;
//    }
    // Уголки
//    ui_scr[PY + 0][PX + 0] = 0xC0;
//    ui_scr[PY +21][PX + 0] = 0xC8;
//    ui_scr[PY + 0][PX +60] = 0xC4;
//    ui_scr[PY +21][PX +60] = 0xCC;
	char str[16];    
    // Рисуем список файлов
    int off = page * FILES * COLS;
    for (size_t i = 0; i < files_count - off && i < FILES * COLS; ++i) {
		int x = PX + 1 + (i / FILES) * FILES;
		int y = PY + 1 + (i % FILES);
        int io = i + off;
		// Имя файла
		ui_draw_text(x + 1, y, files_info[io].name);
		// Размер
        if (files_info[io].fsize) {
            xsprintf(str, "%5d", files_info[io].fsize);
            ui_draw_text(x + 10, y, str);
        } else if (files_info[io].fattrib & AM_DIR) {
		    ui_draw_text(x + 10, y, "<DIR>");
        } else {
            ui_draw_text(x + 10, y, " ??? ");
        }
    }
    // Выбор файла
    while (1) {
        if (n >= files_count - off)
		    n = files_count - off - 1;
		// Стираем курсор с предыдущего файла
		ui_scr[PY + 1 + (prev % FILES)][PX + (prev / FILES) * FILES + 1] = 0x80;
		// Рисуем курсор на новом месте
		ui_scr[PY + 1 + (n % FILES)][PX + (n / FILES) * FILES + 1] = 0x90;
		// Запоминаем текущую позицию
		prev = n;
		// Обрабатываем нажатия кнопок
		while (1) {
	    	uint16_t c = ps2_read();
	    	if ( c == PS2_UP ) {
				// Вверх
				n--;
                if (n < 0) {
                    n = FILES * COLS - 1;
                    if (--page < 0) {
                        page = 0;
                        n = 0;
                    }
                    goto reread;
                }
				break;
	    	} else if ( (c == PS2_DOWN) && (n < files_count - off - 1) ) {
				// Вниз
				n++;
                if (n >= FILES * COLS) {
                    n = 0;
                    page++;
                    goto reread;
                }
				break;
	    	} else if ( c == PS2_LEFT ) {
				// Влево
				n -= FILES;
				if (n < 0) {
                    n += FILES * COLS;
                    if (--page < 0) {
                        page = 0;
                        n = 0;
                    }
                    goto reread;
                }
				break;
	    	} else if ( (c == PS2_RIGHT) && (n < files_count - 1) ) {
				// Вправо
				n += FILES;
				if (n >= files_count) n = files_count - 1;
                if (n >= FILES * COLS) {
                    n -= FILES * COLS;
                    page++;
                    goto reread;
                }
				break;
	    	} else if ( (c == PS2_DELETE) || (c == PS2_D) || (c == PS2_BACKSPACE) ) {
				// Удалить
				del(files_info[n + off].name);
				goto reread;
	    	} else if ( (c == PS2_SPACE) || (c == PS2_R) ) {
				// Переименовать
				rename(files_info[n + off].name);
				goto reread;
	    	} else if ( (c == PS2_ENTER) || (c == PS2_KP_ENTER) ) {
				// Выбрать
				goto done;
	    	} else if (c == PS2_ESC) {
				// Отмена
				n = -1 - off;
				goto done;
	    	}
		}
    }
done:
    // Возвращаем позицию подчеркивания
    screen.underline_y = 7;
    return n + off;
}
