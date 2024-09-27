#include <stdint.h>
#include <map>

static std::map<uint16_t, uint16_t> char2rk;

extern "C" void rk_2_at(uint16_t rk, uint16_t at) {
    char2rk[at] = rk;
}

extern "C" uint16_t rk_by_at(uint16_t at) {
    std::map<uint16_t, uint16_t>::const_iterator i = char2rk.find(at);
    if (i == char2rk.end()) return 0;
    uint16_t res = i->second;
///    printf("%04Xh: %04Xh", at, res);
    return res;
}
