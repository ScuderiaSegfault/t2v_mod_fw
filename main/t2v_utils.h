//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_T2V_UTILS_H
#define T2V_MOD_FW_T2V_UTILS_H
#include <stdint.h>

static uint8_t nibble_to_seven_segment(uint8_t nibble)
{
    switch (nibble & 0x0f)
    {
    case 0x0:
        return 0b00111111;
    case 0x1:
        return 0b00000110;
    case 0x2:
        return 0b11011011;
    case 0x3:
        return 0b11001111;
    case 0x4:
        return 0b01100110;
    case 0x5:
        return 0b11101101;
    case 0x6:
        return 0b11111101;
    case 0x7:
        return 0b00000111;
    case 0x8:
        return 0xef;
    case 0x9:
        return 0b11101111;
    case 0xa:
        return 0b01110111;
    case 0xb:
        return 0b11111100;
    case 0xc:
        return 0b00111001;
    case 0xd:
        return 0b11011110;
    case 0xe:
        return 0b11111001;
    case 0xf:
        return 0b01110001;
    default:
        return 0x00;
    }
}

#endif //T2V_MOD_FW_T2V_UTILS_H