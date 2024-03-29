#include "zkg.h"

#include "ets.h"


uint8_t zkg[4][1024];

const uint8_t zkg_rom[1024]=
{
    // Line 0
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x08, 0x0c, 
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x04, 0x03, 0x00, 
    0x00, 0x04, 0x0a, 0x0a, 0x11, 0x18, 0x04, 0x06, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x0e, 0x04, 0x0e, 0x1f, 0x02, 0x1f, 0x07, 0x1f, 0x0e, 0x0e, 0x00, 0x0c, 0x02, 0x00, 0x08, 0x0e, 
    0x0e, 0x04, 0x1e, 0x0e, 0x1e, 0x1f, 0x1f, 0x0e, 0x11, 0x0e, 0x01, 0x11, 0x10, 0x11, 0x11, 0x0e, 
    0x1e, 0x0e, 0x1e, 0x0e, 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f, 0x0e, 0x00, 0x0e, 0x0e, 0x00, 
    0x12, 0x04, 0x1f, 0x12, 0x06, 0x1f, 0x04, 0x1f, 0x11, 0x11, 0x15, 0x11, 0x07, 0x11, 0x11, 0x0e, 
    0x1f, 0x0f, 0x1e, 0x0e, 0x1f, 0x11, 0x11, 0x1e, 0x10, 0x11, 0x0e, 0x11, 0x0e, 0x15, 0x11, 0x3f, 
    // Line 1
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x0c, 0x00, 0x1e, 0x00, 0x00, 0x0c, 0x0c, 
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x0c, 0x04, 0x00, 
    0x00, 0x04, 0x0a, 0x0a, 0x0e, 0x19, 0x0a, 0x06, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x01, 
    0x11, 0x0c, 0x11, 0x01, 0x06, 0x10, 0x08, 0x01, 0x11, 0x11, 0x0c, 0x0c, 0x04, 0x00, 0x04, 0x11, 
    0x11, 0x0a, 0x11, 0x11, 0x09, 0x10, 0x10, 0x11, 0x11, 0x04, 0x01, 0x12, 0x10, 0x1b, 0x11, 0x11, 
    0x11, 0x11, 0x11, 0x11, 0x04, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x08, 0x10, 0x02, 0x11, 0x00, 
    0x15, 0x0a, 0x10, 0x12, 0x0a, 0x10, 0x1f, 0x11, 0x11, 0x11, 0x11, 0x12, 0x09, 0x1b, 0x11, 0x11, 
    0x11, 0x11, 0x11, 0x11, 0x04, 0x11, 0x15, 0x11, 0x10, 0x11, 0x11, 0x15, 0x11, 0x15, 0x11, 0x3f, 
    // Line 2
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x3f, 0x00, 0x3f, 0x00, 0x00, 0x2e, 0x0c, 
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x1d, 0x16, 0x00, 
    0x00, 0x04, 0x0a, 0x1f, 0x0a, 0x02, 0x0a, 0x02, 0x08, 0x02, 0x15, 0x04, 0x00, 0x00, 0x00, 0x02, 
    0x13, 0x04, 0x01, 0x02, 0x0a, 0x1e, 0x10, 0x02, 0x11, 0x11, 0x0c, 0x00, 0x08, 0x1f, 0x02, 0x01, 
    0x13, 0x11, 0x11, 0x10, 0x09, 0x10, 0x10, 0x10, 0x11, 0x04, 0x01, 0x14, 0x10, 0x15, 0x19, 0x11, 
    0x11, 0x11, 0x11, 0x10, 0x04, 0x11, 0x11, 0x11, 0x0a, 0x0a, 0x02, 0x08, 0x08, 0x02, 0x00, 0x00, 
    0x15, 0x11, 0x10, 0x12, 0x0a, 0x10, 0x15, 0x10, 0x0a, 0x13, 0x13, 0x14, 0x09, 0x15, 0x11, 0x11, 
    0x11, 0x11, 0x11, 0x10, 0x04, 0x11, 0x15, 0x11, 0x10, 0x11, 0x01, 0x15, 0x01, 0x15, 0x11, 0x3f, 
    // Line 3
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x2d, 0x00, 0x0c, 0x00, 0x00, 0x3f, 0x0c, 
    0x00, 0x38, 0x07, 0x3f, 0x00, 0x38, 0x07, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x3f, 0x3f, 0x2d, 0x00, 
    0x00, 0x04, 0x00, 0x0a, 0x0a, 0x04, 0x0c, 0x04, 0x08, 0x02, 0x0e, 0x1f, 0x0c, 0x1f, 0x00, 0x04, 
    0x15, 0x04, 0x06, 0x06, 0x12, 0x01, 0x1e, 0x04, 0x0e, 0x0f, 0x00, 0x0c, 0x10, 0x00, 0x01, 0x02, 
    0x15, 0x11, 0x1e, 0x10, 0x09, 0x1e, 0x1e, 0x10, 0x1f, 0x04, 0x01, 0x18, 0x10, 0x15, 0x15, 0x11, 
    0x1e, 0x11, 0x1e, 0x0e, 0x04, 0x11, 0x0a, 0x15, 0x04, 0x04, 0x0e, 0x08, 0x04, 0x02, 0x00, 0x00, 
    0x1d, 0x11, 0x1e, 0x12, 0x0a, 0x1e, 0x15, 0x10, 0x04, 0x15, 0x15, 0x18, 0x09, 0x15, 0x1f, 0x11, 
    0x11, 0x0f, 0x1e, 0x10, 0x04, 0x0a, 0x0e, 0x1e, 0x1e, 0x19, 0x06, 0x15, 0x07, 0x15, 0x1f, 0x3f, 
    // Line 4
    0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x3f, 0x0c, 
    0x38, 0x38, 0x38, 0x38, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x3f, 0x3f, 0x21, 0x00, 
    0x00, 0x04, 0x00, 0x1f, 0x0e, 0x08, 0x15, 0x00, 0x08, 0x02, 0x15, 0x04, 0x0c, 0x00, 0x00, 0x08, 
    0x19, 0x04, 0x08, 0x01, 0x1f, 0x01, 0x11, 0x08, 0x11, 0x01, 0x00, 0x0c, 0x08, 0x1f, 0x02, 0x04, 
    0x17, 0x1f, 0x11, 0x10, 0x09, 0x10, 0x10, 0x13, 0x11, 0x04, 0x11, 0x14, 0x10, 0x11, 0x13, 0x11, 
    0x10, 0x15, 0x14, 0x01, 0x04, 0x11, 0x0a, 0x15, 0x0a, 0x04, 0x08, 0x08, 0x02, 0x02, 0x00, 0x00, 
    0x15, 0x1f, 0x11, 0x12, 0x0a, 0x10, 0x1f, 0x10, 0x0a, 0x19, 0x19, 0x14, 0x09, 0x11, 0x11, 0x11, 
    0x11, 0x05, 0x10, 0x10, 0x04, 0x04, 0x15, 0x11, 0x11, 0x15, 0x01, 0x15, 0x01, 0x15, 0x01, 0x3f, 
    // Line 5
    0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x2e, 0x3f, 
    0x38, 0x38, 0x38, 0x38, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x1d, 0x21, 0x00, 
    0x00, 0x00, 0x00, 0x0a, 0x11, 0x13, 0x12, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x0c, 0x10, 
    0x11, 0x04, 0x10, 0x11, 0x02, 0x11, 0x11, 0x08, 0x11, 0x02, 0x0c, 0x04, 0x04, 0x00, 0x04, 0x00, 
    0x10, 0x11, 0x11, 0x11, 0x09, 0x10, 0x10, 0x11, 0x11, 0x04, 0x11, 0x12, 0x11, 0x11, 0x11, 0x11, 
    0x10, 0x12, 0x12, 0x11, 0x04, 0x11, 0x04, 0x15, 0x11, 0x04, 0x10, 0x08, 0x01, 0x02, 0x00, 0x00, 
    0x15, 0x11, 0x11, 0x1f, 0x1f, 0x10, 0x04, 0x10, 0x11, 0x11, 0x11, 0x12, 0x09, 0x11, 0x11, 0x11, 
    0x11, 0x09, 0x10, 0x11, 0x04, 0x08, 0x15, 0x11, 0x11, 0x15, 0x11, 0x15, 0x11, 0x1f, 0x01, 0x3f, 
    // Line 6
    0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x12, 0x00, 0x0c, 0x00, 0x00, 0x0c, 0x1e, 
    0x38, 0x38, 0x38, 0x38, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x0c, 0x12, 0x00, 
    0x00, 0x04, 0x00, 0x0a, 0x00, 0x03, 0x0d, 0x00, 0x02, 0x08, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 
    0x0e, 0x0e, 0x1f, 0x0e, 0x02, 0x0e, 0x0e, 0x08, 0x0e, 0x1c, 0x0c, 0x08, 0x02, 0x00, 0x08, 0x04, 
    0x0e, 0x11, 0x1e, 0x0e, 0x1e, 0x1f, 0x10, 0x0f, 0x11, 0x0e, 0x0e, 0x11, 0x1f, 0x11, 0x11, 0x0e, 
    0x10, 0x0d, 0x11, 0x0e, 0x04, 0x0e, 0x04, 0x0a, 0x11, 0x04, 0x1f, 0x0e, 0x00, 0x0e, 0x00, 0x1f, 
    0x12, 0x11, 0x1e, 0x01, 0x11, 0x1f, 0x04, 0x10, 0x11, 0x11, 0x11, 0x11, 0x19, 0x11, 0x11, 0x0e, 
    0x11, 0x11, 0x10, 0x0e, 0x04, 0x10, 0x11, 0x1e, 0x1e, 0x19, 0x0e, 0x1f, 0x0e, 0x01, 0x00, 0x3f, 
    // Line 7
    0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x21, 0x00, 0x0c, 0x00, 0x00, 0x08, 0x0c, 
    0x38, 0x38, 0x38, 0x38, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x04, 0x0c, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 
};
