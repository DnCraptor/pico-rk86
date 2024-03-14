#include "ets.h"

void ets_memset(void* b, uint8_t v, uint32_t sz) {
    memset(b, v, sz);
}

void ets_memcpy(void* t, const void* f, uint32_t sz) {
    memcpy(t, f, sz);
}
