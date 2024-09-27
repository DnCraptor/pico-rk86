#include "debug.h"
#include "ff.h"
#include <string.h>

static FIL f;
void logMsg(char* msg) {
    FRESULT fr = f_open(&f, DEBUG_LOG, FA_WRITE | FA_OPEN_APPEND | FA_OPEN_ALWAYS);
    if (fr != FR_OK) return;
    UINT bw;
    f_write(&f, msg, strlen(msg), &bw);
    f_write(&f, "\n", 1, &bw);
    f_close(&f);
}
