/**
  ******************************************************************************
  * @file    watchdog.h
  * @brief   Independent watchdog (IWDG) that stops a hung program.
  *
  * The IWDG is its own peripheral clocked by the internal ~40 kHz LSI, so it
  * uses no timer and keeps running even if the main clock or the program dies.
  * A reset also drops TIM2, which takes every motor output with it.
  ******************************************************************************
  */
#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Nominal timeout. The LSI is only specified as 30..60 kHz, so the real
 * timeout lies between about 2/3 and 4/3 of this. */
#define WATCHDOG_TIMEOUT_MS   120U

/* Starts the watchdog. It cannot be stopped afterwards, so call it once the
 * slow start-up (sensor reset, waits) is finished and the loop is running.
 * The IWDG is frozen while a debugger halts the core. */
void Watchdog_Start(void);

/* Call once per main-loop pass. */
void Watchdog_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H__ */
