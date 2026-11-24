/*
 * DroneCAN.h — узел DroneCAN (UAVCAN v0) ESC. Порт ref/bldc/libcanard/canard_driver.c
 * (VESC) на наш контроллер: libcanard + DSDL из third_party/libcanard.
 *
 * Принимает: esc.RawCommand (1030), esc.RPMCommand (1031);
 *            сервисы GetNodeInfo, RestartNode, param.GetSet, param.ExecuteOpcode.
 * Передаёт:  uavcan.protocol.NodeStatus (1 Гц), esc.Status (can_status_ms).
 * Параметры узла — в TFocParams (can_*), правятся и по UART, и по param.GetSet.
 */
#ifndef DRONECAN_H
#define DRONECAN_H

#include <stdbool.h>
#include <stdint.h>

#define DRONECAN_NODE_NAME "foc.k1946vk035"
#define DRONECAN_FW_MAJOR 0u
#define DRONECAN_FW_MINOR 1u

/* Старт узла по g_params (can_node_id = 0 -> узел выключен). */
bool dronecan_init(void);
/* Главный цикл: приём, обработчики, периодические сообщения, передача. */
void dronecan_poll(uint32_t now_ms);
bool dronecan_enabled(void);

typedef struct {
  uint32_t rx_transfers;
  uint32_t tx_frames;
  uint32_t tx_dropped; /* зависшая передача (нет ACK) */
  uint32_t cmd_count;  /* принятые команды управления */
  uint32_t timeouts;   /* срабатываний таймаута команды */
  uint8_t can_active;  /* 1: двигателем сейчас управляет CAN */
} dronecan_stats_t;
void dronecan_stats(dronecan_stats_t *s);

#endif /* DRONECAN_H */
