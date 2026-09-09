/**
  ******************************************************************************
  * @file    display.h
  * @brief   Status page on the bit-banged OLED.
  ******************************************************************************
  */
#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Refresh period. The panel is on a bit-banged bus and nothing here is time
 * critical, so this is deliberately slow. */
#define DISPLAY_PERIOD_MS      100U

void Display_Init(void);

/* Redraws the whole page from the shared state and pushes it to the panel. */
void Display_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_H__ */
