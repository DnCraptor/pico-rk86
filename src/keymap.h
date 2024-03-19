#ifndef KEYMAP_H
#define KEYMAP_H


#ifdef __cplusplus
extern "C" {
#endif


#include "ets.h"

typedef struct kmap
{
    uint16_t ps2;
    uint16_t rk;
} kmap;

void keymap_init(void);
uint16_t keymap_periodic(void);


#ifdef __cplusplus
};
#endif


#endif
