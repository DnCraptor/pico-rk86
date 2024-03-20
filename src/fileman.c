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

int16_t fileman(uint8_t type, const char *text)
{
    DIR dir;
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
    // Сортируем каталог по имени
	qsort (files_info, files_count, sizeof(file_info_t), m_comp);
    // Рисуем
    ui_clear();
    ui_header("RADIO-86RK -->");
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
    for (size_t i = 0; i < files_count; i++) {
		int x = PX + 1 + (i / 17) * 17;
		int y = PY + 1 + (i % 17);
		// Имя файла
		ui_draw_text(x + 1, y, files_info[i].name);
		// Размер
		xsprintf(str, "%5d", files_info[i].fsize);
		ui_draw_text(x + 10, y, str);
    }
    int n = 0, prev = 0;
    // Выбор файла
    while (1) {
        if (n >= files_count)
		    n = files_count - 1;
		// Стираем курсор с предыдущего файла
		ui_scr[PY + 1 + (prev % 17)][PX + (prev / 17) * 17 + 1] = 0x80;
		// Рисуем курсор на новом месте
		ui_scr[PY + 1 + (n % 17)][PX + (n / 17) * 17 + 1] = 0x90;
		// Запоминаем текущую позицию
		prev = n;
		// Обрабатываем нажатия кнопок
		while (1) {
	    	uint16_t c = ps2_read();
	    	if ( (c == PS2_UP) && (n > 0) ) {
				// Вверх
				n--;
				break;
	    	} else if ( (c == PS2_DOWN) && (n < files_count - 1) ) {
				// Вниз
				n++;
				break;
	    	} else if ( (c == PS2_LEFT) && (n > 0) ) {
				// Влево
				n -= 18;
				if (n < 0) n = 0;
				break;
	    	} else if ( (c == PS2_RIGHT) && (n < files_count - 1) ) {
				// Вправо
				n += 18;
				if (n >= files_count) n = files_count - 1;
				break;
	    	} else if ( (c == PS2_DELETE) || (c == PS2_D) || (c == PS2_BACKSPACE) ) {
				// Удалить
				del(files_info[n].name);
				goto reread;
	    	} else if ( (c == PS2_SPACE) || (c == PS2_R) ) {
				// Переименовать
				rename(files_info[n].name);
				goto reread;
	    	} else if ( (c == PS2_ENTER) || (c == PS2_KP_ENTER) ) {
				// Выбрать
				goto done;
	    	} else if (c == PS2_ESC) {
				// Отмена
				n = -1;
				goto done;
	    	}
		}
    }
done:
    // Возвращаем позицию подчеркивания
    screen.underline_y = 7;
    return n;
}
