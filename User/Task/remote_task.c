/**
  ******************************************************************************
  * @file    remote_task.c
  * @brief   nRF24L01 receive chain.
  *
  * The radio wiring is supplied through RemoteTask_Init, so SPI instance and
  * CE/CSN pin assignment stay outside this file.
  ******************************************************************************
  */

#include "remote_task.h"

/* All data access runs in the main loop; EXTI only sets Remote_Ready. */

#include <string.h>

/* Payload layout, little endian. Bytes 13..19 are still spare. */
#define REMOTE_OFFSET_ROLL            0U
#define REMOTE_OFFSET_PITCH           2U
#define REMOTE_OFFSET_YAW             4U
#define REMOTE_OFFSET_THROTTLE        6U
#define REMOTE_OFFSET_PARAM_ID        8U
#define REMOTE_OFFSET_PARAM_VALUE     9U

static NRF24_RX_Handle_t remote_nrf;
static RemoteControl_t remote_control;
static volatile uint8_t remote_initialized;
/* Latches once the transmitter has been seen at idle throttle. */
static volatile uint8_t remote_throttle_idle_seen;

static void RemoteTask_Parse(const uint8_t *payload);
static uint16_t RemoteTask_ReadUint16(const uint8_t *data);
static float RemoteTask_ReadFloat(const uint8_t *data);
static int16_t RemoteTask_ReadInt16(const uint8_t *data);
static int16_t RemoteTask_ClampAxis(int16_t value);

HAL_StatusTypeDef RemoteTask_Init(const NRF24_RX_Handle_t *config)
{
  if (config == NULL)
  {
    return HAL_ERROR;
  }

  remote_initialized = 0U;
  remote_throttle_idle_seen = 0U;
  remote_nrf = *config;
  memset(&remote_control, 0, sizeof(remote_control));
  remote_control.param_id = REMOTE_PARAM_NONE;

  /* The IRQ pin and its NVIC entry belong to MX_GPIO_Init. */
  if (NRF24_RX_Init(&remote_nrf) != HAL_OK)
  {
    return HAL_ERROR;
  }

  remote_initialized = 1U;
  return HAL_OK;
}

void RemoteTask_Update(void)
{
  uint8_t is_ready = 0U;
  uint8_t drained = 0U;
  uint8_t failed = 0U;

  if (remote_initialized == 0U)
  {
    return;
  }

  /* One falling edge can cover several queued frames, because the radio holds
   * IRQ low until RX_DR is cleared. Drain the FIFO rather than read once. */
  for (;;)
  {
    uint8_t payload[NRF24_RX_PAYLOAD_SIZE];

    if (NRF24_RX_IsDataReady(&remote_nrf, &is_ready) != HAL_OK)
    {
      failed = 1U;
      break;
    }
    if (is_ready == 0U)
    {
      break;
    }
    if (NRF24_RX_Read(&remote_nrf, payload) != HAL_OK)
    {
      failed = 1U;
      break;
    }

    RemoteTask_Parse(payload);
    drained = 1U;
  }

  /* The handoff is edge triggered, so leaving IRQ asserted kills the link for
   * good: no further falling edge would ever arrive. Bailing out on an SPI
   * error does exactly that, and so does a flag other than RX_DR latching with
   * an empty FIFO. Either way, clearing the flags releases the line. */
  if ((failed != 0U) ||
      ((drained == 0U) &&
       (HAL_GPIO_ReadPin(REMOTE_NRF24_IRQ_PORT, REMOTE_NRF24_IRQ_PIN) == GPIO_PIN_RESET)))
  {
    (void)NRF24_RX_ClearInterrupts(&remote_nrf);
  }
}

void RemoteTask_GetControl(RemoteControl_t *control)
{
  if (control == NULL)
  {
    return;
  }

  *control = remote_control;
}

uint8_t RemoteTask_IsLinkUp(void)
{
  uint32_t frame_count;
  uint32_t last_tick_ms;

  frame_count = remote_control.frame_count;
  last_tick_ms = remote_control.last_tick_ms;

  /* A zero timestamp is indistinguishable from a tick early after boot, so the
   * frame counter is what tells "no frame yet" apart from "a frame at tick 0". */
  if (frame_count == 0U)
  {
    return 0U;
  }

  /* Unsigned arithmetic, so the 49-day tick wrap takes care of itself. */
  return ((HAL_GetTick() - last_tick_ms) <= REMOTE_LINK_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t RemoteTask_IsControlAllowed(void)
{
  if (RemoteTask_IsLinkUp() == 0U)
  {
    /* Arming has to be earned again after every outage, otherwise a link that
     * drops and returns with the stick up spins the motors immediately. */
    remote_throttle_idle_seen = 0U;
    return 0U;
  }

  return remote_throttle_idle_seen;
}

static void RemoteTask_Parse(const uint8_t *payload)
{
  const int16_t roll = RemoteTask_ClampAxis(
      RemoteTask_ReadInt16(&payload[REMOTE_OFFSET_ROLL]));
  const int16_t pitch = RemoteTask_ClampAxis(
      RemoteTask_ReadInt16(&payload[REMOTE_OFFSET_PITCH]));
  const int16_t yaw = RemoteTask_ClampAxis(
      RemoteTask_ReadInt16(&payload[REMOTE_OFFSET_YAW]));
  const uint8_t param_id = payload[REMOTE_OFFSET_PARAM_ID];
  const float param_value = RemoteTask_ReadFloat(&payload[REMOTE_OFFSET_PARAM_VALUE]);
  uint16_t throttle = RemoteTask_ReadUint16(&payload[REMOTE_OFFSET_THROTTLE]);

  if (throttle > (uint16_t)REMOTE_THROTTLE_MAX)
  {
    throttle = (uint16_t)REMOTE_THROTTLE_MAX;
  }

  if (throttle <= (uint16_t)REMOTE_ARM_THROTTLE_MAX)
  {
    remote_throttle_idle_seen = 1U;
  }

  remote_control.roll = roll;
  remote_control.pitch = pitch;
  remote_control.yaw = yaw;
  remote_control.throttle = throttle;
  remote_control.param_id = param_id;
  remote_control.param_value = param_value;
  remote_control.frame_count++;
  remote_control.last_tick_ms = HAL_GetTick();
}

/* Assembled byte by byte so the wire layout does not depend on struct packing
 * or alignment at either end of the link. */
static uint16_t RemoteTask_ReadUint16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static float RemoteTask_ReadFloat(const uint8_t *data)
{
  const uint32_t bits = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                        ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
  float value;

  /* memcpy rather than a pointer cast: the payload buffer has no alignment
   * guarantee and type punning through a pointer is undefined behaviour. */
  (void)memcpy(&value, &bits, sizeof(value));
  return value;
}

static int16_t RemoteTask_ReadInt16(const uint8_t *data)
{
  return (int16_t)RemoteTask_ReadUint16(data);
}

static int16_t RemoteTask_ClampAxis(int16_t value)
{
  if (value < (int16_t)REMOTE_AXIS_MIN)
  {
    return (int16_t)REMOTE_AXIS_MIN;
  }
  if (value > (int16_t)REMOTE_AXIS_MAX)
  {
    return (int16_t)REMOTE_AXIS_MAX;
  }
  return value;
}
