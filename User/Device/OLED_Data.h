#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

#define OLED_CHARSET_UTF8
/* #define OLED_CHARSET_GB2312 */

typedef struct
{
#ifdef OLED_CHARSET_UTF8
    char Index[5];
#endif

#ifdef OLED_CHARSET_GB2312
    char Index[3];
#endif

    uint8_t Data[32];
} ChineseCell_t;

extern const uint8_t OLED_F8x16[][16];
extern const uint8_t OLED_F6x8[][6];
extern const ChineseCell_t OLED_CF16x16[];
extern const uint8_t Diode[];

#endif