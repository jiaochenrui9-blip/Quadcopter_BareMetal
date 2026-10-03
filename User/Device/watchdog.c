#include "watchdog.h"

#define IWDG_KEY_RELOAD    0xAAAAU
#define IWDG_KEY_ENABLE    0xCCCCU
#define IWDG_KEY_UNLOCK    0x5555U
#define IWDG_PRESCALER_32  0x03U
/* LSI 40 kHz / 32 = 1250 counts per second, so 1.25 counts per millisecond. */
#define IWDG_RELOAD        ((WATCHDOG_TIMEOUT_MS * 5U) / 4U)
#define IWDG_UPDATE_WAIT_MS 48U

void Watchdog_Start(void)
{
  const uint32_t start = HAL_GetTick();

  __HAL_DBGMCU_FREEZE_IWDG();

  /* Same order as the HAL: start (which also starts the LSI), unlock the
   * configuration registers, write them, then wait for the slow LSI domain
   * to latch the new values. A timeout here is ignored on purpose: the
   * watchdog is already running with the reset defaults, which is still a
   * working, if slower, safety net. */
  IWDG->KR = IWDG_KEY_ENABLE;
  IWDG->KR = IWDG_KEY_UNLOCK;
  IWDG->PR = IWDG_PRESCALER_32;
  IWDG->RLR = IWDG_RELOAD;
  while (IWDG->SR != 0U)
  {
    if ((HAL_GetTick() - start) > IWDG_UPDATE_WAIT_MS)
    {
      break;
    }
  }
  IWDG->KR = IWDG_KEY_RELOAD;
}

void Watchdog_Refresh(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}
