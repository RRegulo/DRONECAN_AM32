/* DroneCAN.h — транспорт libcanard для К1946ВК035. GNU GPLv3. */
#ifndef DRONECAN_H
#define DRONECAN_H

#include <stdbool.h>
#include <stdint.h>
#include "canard.h"

typedef struct {
  uint32_t tx_frames;
  uint32_t tx_dropped;
} dronecan_transport_stats_t;

/* Прикладной код задаёт ID и обработчики libcanard; порт не управляет мотором. */
bool dronecan_transport_init(uint16_t bitrate_kbps, uint8_t node_id,
                             CanardOnTransferReception on_rx,
                             CanardShouldAcceptTransfer accept, void *user);
CanardInstance *dronecan_transport_instance(void);
void dronecan_transport_poll(uint32_t now_ms);
void dronecan_transport_stats(dronecan_transport_stats_t *stats);

#endif
