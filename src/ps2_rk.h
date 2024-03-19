#ifndef PS2_DRV_H
#define PS2_DRV_H


#include "ets.h"
#include "ps2_codes_rk.h"


#ifdef __cplusplus
extern "C" {
#endif


void ps2_init(void);

uint16_t ps2_read(void);
void ps2_leds(bool caps, bool num, bool scroll);


#ifdef __cplusplus
};
#endif


#endif
