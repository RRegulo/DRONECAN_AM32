/* DroneCAN.c — обмен между CAN0 и libcanard, без прикладной логики. GNU GPLv3. */
#include "DroneCAN.h"
#include "CAN_Driver.h"
#include <string.h>

#define POOL_BYTES 960u
#define TX_STUCK_MS 20u

static CanardInstance s_ins;
static uint8_t s_pool[POOL_BYTES] __attribute__((aligned(8)));
static dronecan_transport_stats_t s_stats;
static uint32_t s_cleanup_ms, s_tx_since;
static bool s_enabled, s_tx_waiting;

bool dronecan_transport_init(uint16_t bitrate_kbps, uint8_t node_id,
                             CanardOnTransferReception on_rx,
                             CanardShouldAcceptTransfer accept, void *user) {
  s_enabled = false;
  if (node_id == 0u || node_id > 127u || on_rx == NULL || accept == NULL ||
      !can_hw_init(bitrate_kbps, node_id, false))
    return false;
  memset(&s_stats, 0, sizeof(s_stats));
  s_cleanup_ms = 0u;
  s_tx_waiting = false;
  canardInit(&s_ins, s_pool, sizeof(s_pool), on_rx, accept, user);
  canardSetLocalNodeID(&s_ins, node_id);
  s_enabled = true;
  return true;
}

CanardInstance *dronecan_transport_instance(void) {
  return &s_ins;
}

void dronecan_transport_stats(dronecan_transport_stats_t *stats) {
  *stats = s_stats;
}

void dronecan_transport_poll(uint32_t now_ms) {
  if (!s_enabled)
    return;
  can_hw_poll();
  can_frame_t f;
  for (uint32_t n = 0u; n < 16u && can_hw_rx_pop(&f); n++) {
    if (f.dlc > 8u)
      continue;
    CanardCANFrame cf;
    memset(&cf, 0, sizeof(cf));
    cf.id = f.id | CANARD_CAN_FRAME_EFF;
    cf.data_len = f.dlc;
    memcpy(cf.data, f.data, 8u);
    (void)canardHandleRxFrame(&s_ins, &cf, (uint64_t)now_ms * 1000u);
  }
  if ((now_ms - s_cleanup_ms) >= 1000u) {
    s_cleanup_ms = now_ms;
    canardCleanupStaleTransfers(&s_ins, (uint64_t)now_ms * 1000u);
  }
  for (uint32_t n = 0u; n < 8u; n++) {
    const CanardCANFrame *cf = canardPeekTxQueue(&s_ins);
    if (cf == NULL) {
      s_tx_waiting = false;
      break;
    }
    can_frame_t tx;
    tx.id = cf->id & CANARD_CAN_EXT_ID_MASK;
    tx.dlc = cf->data_len;
    memcpy(tx.data, cf->data, 8u);
    if (can_hw_tx(&tx)) {
      canardPopTxQueue(&s_ins);
      s_stats.tx_frames++;
      s_tx_waiting = false;
    } else {
      if (!s_tx_waiting) {
        s_tx_waiting = true;
        s_tx_since = now_ms;
      } else if ((now_ms - s_tx_since) > TX_STUCK_MS) {
        can_hw_tx_abort();
        canardPopTxQueue(&s_ins);
        s_stats.tx_dropped++;
        s_tx_waiting = false;
      }
      break;
    }
  }
}
