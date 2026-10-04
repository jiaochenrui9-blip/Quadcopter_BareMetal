/**
  ******************************************************************************
  * @file    remote_task.h
  * @brief   nRF24L01 receive chain: IRQ line, task handoff, raw frame storage.
  ******************************************************************************
  */
#ifndef __REMOTE_TASK_H__
#define __REMOTE_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf24_rx.h"

/* nRF24L01 IRQ line. The radio drives it low on RX_DR, so the EXTI triggers
 * on the falling edge. */
#define REMOTE_NRF24_IRQ_PORT      NRF_IRQ_GPIO_Port
#define REMOTE_NRF24_IRQ_PIN       NRF_IRQ_Pin
#define REMOTE_NRF24_IRQ_IRQn      NRF_IRQ_EXTI_IRQn

/* The link counts as lost after this long without an accepted frame. */
#define REMOTE_LINK_TIMEOUT_MS     200U

/* Throttle at or below this counts as idle, which is what arms the aircraft. */
#define REMOTE_ARM_THROTTLE_MAX    20U

/* Stick ranges the transmitter is expected to stay within. */
#define REMOTE_AXIS_MIN            (-1000)
#define REMOTE_AXIS_MAX            (1000)
#define REMOTE_THROTTLE_MIN        (0)
#define REMOTE_THROTTLE_MAX        (1000)

/* Parameter slot. The transmitter repeats the same id and value in every frame
 * for as long as the knob sits there, so a lost frame costs nothing and no
 * sequence numbers or retries are needed. */
#define REMOTE_PARAM_NONE          0xFFU

typedef struct
{
  int16_t roll;            /* REMOTE_AXIS_MIN .. REMOTE_AXIS_MAX */
  int16_t pitch;           /* REMOTE_AXIS_MIN .. REMOTE_AXIS_MAX */
  int16_t yaw;             /* REMOTE_AXIS_MIN .. REMOTE_AXIS_MAX */
  uint16_t throttle;       /* REMOTE_THROTTLE_MIN .. REMOTE_THROTTLE_MAX */

  uint8_t param_id;        /* REMOTE_PARAM_NONE when nothing is being tuned */
  float param_value;

  uint32_t frame_count;    /* frames accepted since boot */
  uint32_t last_tick_ms;   /* HAL tick of the most recent frame */
} RemoteControl_t;

extern RemoteControl_t g_remote_control;

/* Takes the radio wiring, configures the IRQ line and brings the radio up.
 * Pin and SPI assignment stays with the caller. */
HAL_StatusTypeDef RemoteTask_Init(const NRF24_RX_Handle_t *config);

/* Drains the nRF24 RX FIFO and parses each frame into the shared control set. */
void RemoteTask_Update(void);

void RemoteTask_GetControl(RemoteControl_t *control);

/* Zero once REMOTE_LINK_TIMEOUT_MS has passed with no frame, and before the
 * first frame ever arrives. */
uint8_t RemoteTask_IsLinkUp(void);

/* Link up AND the throttle seen at idle at least once since the last outage.
 * This is the predicate the control loop should gate on. */
uint8_t RemoteTask_IsControlAllowed(void);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_TASK_H__ */
