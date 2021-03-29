#ifndef __FONT_H
#define __FONT_H

#include "stdint.h"

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

//Font lib.
const FontDef Font_7x10;
const FontDef Font_11x18;
const FontDef Font_16x26;

#endif
