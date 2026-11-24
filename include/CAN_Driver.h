/*
 * CAN_Driver.h — драйвер CAN0 К1921ВК035 (MultiCAN-подобный: узлы, списки,
 * объекты сообщений). Только extended-кадры (DroneCAN, 29 бит).
 *
 * Пины платы (схема rev1.0): CAN0_TX = PB15, CAN0_RX = PB14 -> TJA1042
 * (STB на землю), на плате терминатор 120 Ом (R9), разъём CN7.
 *
 * Приём: аппаратные фильтры на объектах 1..3 узла CAN0 -> прерывание
 * (CAN2_IRQn) -> кольцевая очередь; передача — один объект, неблокирующая.
 * Самотест — на внутренней виртуальной шине CAN0<->CAN1 (режим обратной
 * петли), без внешней шины и трансивера.
 */
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t id; /* 29-битный идентификатор (extended) */
  uint8_t dlc; /* 0..8 */
  uint8_t data[8];
} can_frame_t;

/* Инициализация узла CAN0: битрейт 125/250/500/1000 кбит/с. node_id — наш узел
 * DroneCAN (фильтр сервисных кадров), 0 — сервисный фильтр выключен. */
bool can_hw_init(uint16_t bitrate_kbps, uint8_t node_id, bool loopback);
/* Сменить node_id в фильтре сервисных кадров (после назначения узла). */
void can_hw_set_node_id(uint8_t node_id);

/* Передача: false — объект занят (предыдущий кадр ещё уходит). */
bool can_hw_tx(const can_frame_t *f);
/* Снять зависшую передачу (нет ACK на пустой шине). */
void can_hw_tx_abort(void);
bool can_hw_tx_busy(void);

/* Приём из очереди (заполняется прерыванием). */
bool can_hw_rx_pop(can_frame_t *f);

/* Состояние: bit0 BOFF, bit1 EWRN, [4:2] LEC (код последней ошибки);
 * счётчики потерь/переполнений — через can_hw_stats. */
typedef struct {
  uint32_t rx_frames;
  uint32_t rx_lost;     /* MSGLST объекта или переполнение очереди */
  uint32_t tx_frames;
  uint32_t tx_aborts;
  uint32_t bus_off_recoveries;
  uint8_t last_lec;
} can_stats_t;
void can_hw_stats(can_stats_t *s);
/* Вызывать из главного цикла: восстановление после bus-off. */
void can_hw_poll(void);

/* Самотест на виртуальной шине. Возвращает битовую маску пройденных тестов:
 * bit0 CAN0->CAN1 (TX), bit1 CAN1->CAN0 RawCommand-фильтр (RX+ISR+очередь),
 * bit2 фильтр RPMCommand, bit3 сервисный фильтр (узел), bit4 чужой кадр
 * отброшен, bit5 данные/длина сохраняются. Полный успех = 0x3F.
 * Переинициализирует драйвер (нормальный режим восстанавливается). */
#define CAN_SELFTEST_ALL 0x3Fu
uint32_t can_hw_selftest(uint16_t bitrate_kbps, uint8_t node_id);

/* Линия прерывания приёма (NVIC CANn_IRQn) */
#define CAN_RX_IRQ_LINE 2u

#endif /* CAN_DRIVER_H */
