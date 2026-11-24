/*
 * CAN_Driver.c — драйвер CAN0 К1921ВК035. Порядок инициализации и семантика
 * регистров — по РП (разд. 18.12 «Программирование контроллера CAN») и SDK
 * (projects/NIIET-DEV-K1921VK035/can, can_fifo).
 *
 * Объекты сообщений узла CAN0 (список 1):
 *   0 — передача;  1 — приём RawCommand;  2 — приём RPMCommand;
 *   3 — приём сервисных кадров, адресованных нашему узлу.
 * Узел CAN1 (список 2) используется только самотестом (объекты 10, 11).
 */
#include "CAN_Driver.h"

#include <string.h>

#include "K1921VK035.h"

#define CAN_OBJ_TX 0u
#define CAN_OBJ_RX_RAW 1u
#define CAN_OBJ_RX_RPM 2u
#define CAN_OBJ_RX_SVC 3u
#define CAN_OBJ_T1_TX 10u /* самотест: передача узла CAN1 */
#define CAN_OBJ_T1_RX 11u /* самотест: приём всего узлом CAN1 */

/* DroneCAN v0: тип сообщения [23:8], бит 7 = service-not-message, источник [6:0];
 * сервис: тип [23:16], req/resp [15], получатель [14:8], бит 7 = 1. */
#define DC_ID_RAWCOMMAND (1030ul << 8)
#define DC_ID_RPMCOMMAND (1031ul << 8)
#define DC_MASK_MSG_TYPE 0x00FFFF80ul /* тип + бит service */
#define DC_MASK_SVC_DEST 0x00007F80ul /* получатель + бит service */

#define RXQ_LEN 16u

static volatile can_frame_t s_rxq[RXQ_LEN];
static volatile uint32_t s_rx_head; /* пишет ISR */
static volatile uint32_t s_rx_tail; /* читает main */
static volatile can_stats_t s_stats;
static uint32_t s_tx_busy_since;
static uint16_t s_bitrate = 1000u;
static uint8_t s_node_id;
static bool s_loopback;

/* --- вспомогательное ------------------------------------------------------ */

static void obj_to_list(uint32_t obj, uint32_t list) {
  CAN->PANCTR = (0x2u << CAN_PANCTR_PANCMD_Pos) | (obj << CAN_PANCTR_PANAR1_Pos) |
                (list << CAN_PANCTR_PANAR2_Pos);
  while (CAN->PANCTR_bit.BUSY || CAN->PANCTR_bit.RBUSY) {
  }
}

/* Подбор NBTR: Ntq*(BRP+1) = fCAN/bitrate, точка выборки ~85%. */
static bool calc_nbtr(uint32_t kbps, uint32_t *nbtr) {
  uint32_t total = (SystemCoreClock / 1000u) / kbps; /* тактов на бит (FDR 1:1) */
  uint32_t best = 0u, best_err = 0xFFFFFFFFu;
  for (uint32_t ntq = 25u; ntq >= 8u; ntq--) {
    if (total % ntq != 0u)
      continue;
    uint32_t brp1 = total / ntq;
    if (brp1 < 1u || brp1 > 64u)
      continue;
    uint32_t tseg2 = (ntq * 15u + 50u) / 100u; /* ~15% бита */
    if (tseg2 < 2u)
      tseg2 = 2u;
    if (tseg2 > 8u)
      tseg2 = 8u;
    uint32_t tseg1 = ntq - 1u - tseg2;
    if (tseg1 < 3u || tseg1 > 16u)
      continue;
    uint32_t sp = (1000u * (1u + tseg1)) / ntq; /* точка выборки, промилле */
    uint32_t err = sp > 850u ? sp - 850u : 850u - sp;
    if (err < best_err) {
      best_err = err;
      uint32_t sjw = tseg2 - 1u;
      if (sjw > 3u)
        sjw = 3u;
      best = ((tseg2 - 1u) << CAN_Node_NBTR_TSEG2_Pos) |
             ((tseg1 - 1u) << CAN_Node_NBTR_TSEG1_Pos) | (sjw << CAN_Node_NBTR_SJW_Pos) |
             ((brp1 - 1u) << CAN_Node_NBTR_BRP_Pos);
    }
  }
  if (best == 0u)
    return false;
  *nbtr = best;
  return true;
}

static void node_config(uint32_t n, uint32_t nbtr, bool lbm) {
  CAN->Node[n].NCR = CAN_Node_NCR_CCE_Msk | CAN_Node_NCR_INIT_Msk;
  CAN->Node[n].NPCR = lbm ? CAN_Node_NPCR_LBM_Msk : 0u;
  CAN->Node[n].NBTR = nbtr;
  CAN->Node[n].NCR = 0u; /* INIT и CCE сброшены: узел на шине */
}

/* Объект приёма с маской: приём только extended-кадров. */
static void obj_rx_config(uint32_t o, uint32_t id, uint32_t mask, uint32_t irq_line) {
  CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESMSGVAL_Msk; /* реконфигурация при MSGVAL=0 */
  CANMSG->Msg[o].MOAR = (0x2u << CANMSG_Msg_MOAR_PRI_Pos) | CANMSG_Msg_MOAR_IDE_Msk |
                        (id & CANMSG_Msg_MOAR_ID_Msk);
  CANMSG->Msg[o].MOAMR = CANMSG_Msg_MOAMR_MIDE_Msk | (mask & CANMSG_Msg_MOAMR_AM_Msk);
  CANMSG->Msg[o].MOFCR = (0x8u << CANMSG_Msg_MOFCR_DLC_Pos) | CANMSG_Msg_MOFCR_RXIE_Msk;
  CANMSG->Msg[o].MOIPR = (irq_line << CANMSG_Msg_MOIPR_RXINP_Pos);
  CANMSG->Msg[o].MODATAL = 0u;
  CANMSG->Msg[o].MODATAH = 0u;
  /* DIR=0 (приём), RXEN, сбросить флаги, MSGVAL */
  CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESDIR_Msk | CANMSG_Msg_MOCTR_RESNEWDAT_Msk |
                         CANMSG_Msg_MOCTR_RESRXPND_Msk | CANMSG_Msg_MOCTR_RESMSGLST_Msk |
                         CANMSG_Msg_MOCTR_SETRXEN_Msk | CANMSG_Msg_MOCTR_SETMSGVAL_Msk;
}

static void obj_tx_config(uint32_t o) {
  CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESMSGVAL_Msk;
  CANMSG->Msg[o].MOAR = (0x2u << CANMSG_Msg_MOAR_PRI_Pos) | CANMSG_Msg_MOAR_IDE_Msk;
  CANMSG->Msg[o].MOFCR = (0x8u << CANMSG_Msg_MOFCR_DLC_Pos);
  CANMSG->Msg[o].MOIPR = 0u;
  /* DIR=1 (передача), TXEN0/TXEN1; MSGVAL и TXRQ ставим на каждый кадр */
  CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_SETDIR_Msk | CANMSG_Msg_MOCTR_SETTXEN0_Msk |
                         CANMSG_Msg_MOCTR_SETTXEN1_Msk | CANMSG_Msg_MOCTR_RESTXRQ_Msk;
}

static void obj_tx_send(uint32_t o, const can_frame_t *f) {
  CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESMSGVAL_Msk;
  CANMSG->Msg[o].MOAR = (0x2u << CANMSG_Msg_MOAR_PRI_Pos) | CANMSG_Msg_MOAR_IDE_Msk |
                        (f->id & CANMSG_Msg_MOAR_ID_Msk);
  CANMSG->Msg[o].MOFCR = (CANMSG->Msg[o].MOFCR & ~CANMSG_Msg_MOFCR_DLC_Msk) |
                         ((uint32_t)(f->dlc & 0xFu) << CANMSG_Msg_MOFCR_DLC_Pos);
  uint32_t lo = (uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8) |
                ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24);
  uint32_t hi = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8) |
                ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24);
  CANMSG->Msg[o].MODATAL = lo;
  CANMSG->Msg[o].MODATAH = hi;
  CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_SETTXRQ_Msk | CANMSG_Msg_MOCTR_SETMSGVAL_Msk;
}

/* Чтение принятого кадра из объекта по РП: NEWDAT=0, читаем, проверяем
 * NEWDAT/RXUPD, при несоответствии — заново. */
static bool obj_rx_read(uint32_t o, can_frame_t *f) {
  for (uint32_t tries = 0; tries < 4u; tries++) {
    if (!CANMSG->Msg[o].MOSTAT_bit.NEWDAT)
      return false;
    CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESNEWDAT_Msk;
    uint32_t ar = CANMSG->Msg[o].MOAR;
    uint32_t fcr = CANMSG->Msg[o].MOFCR;
    uint32_t lo = CANMSG->Msg[o].MODATAL;
    uint32_t hi = CANMSG->Msg[o].MODATAH;
    if (CANMSG->Msg[o].MOSTAT_bit.NEWDAT || CANMSG->Msg[o].MOSTAT_bit.RXUPD)
      continue; /* в момент чтения пришёл новый кадр */
    f->id = ar & CANMSG_Msg_MOAR_ID_Msk;
    uint32_t dlc = (fcr & CANMSG_Msg_MOFCR_DLC_Msk) >> CANMSG_Msg_MOFCR_DLC_Pos;
    f->dlc = (uint8_t)(dlc > 8u ? 8u : dlc);
    for (uint32_t i = 0; i < 4u; i++) {
      f->data[i] = (uint8_t)(lo >> (8u * i));
      f->data[4u + i] = (uint8_t)(hi >> (8u * i));
    }
    return true;
  }
  return false;
}

static void rxq_push(const can_frame_t *f) {
  uint32_t next = (s_rx_head + 1u) % RXQ_LEN;
  if (next == s_rx_tail) {
    s_stats.rx_lost++;
    return;
  }
  s_rxq[s_rx_head].id = f->id;
  s_rxq[s_rx_head].dlc = f->dlc;
  for (uint32_t i = 0; i < 8u; i++)
    s_rxq[s_rx_head].data[i] = f->data[i];
  s_rx_head = next;
  s_stats.rx_frames++;
}

/* Обработчик линии приёма: обходим наши объекты приёма. */
void CAN2_IRQHandler(void) {
  can_frame_t f;
  for (uint32_t o = CAN_OBJ_RX_RAW; o <= CAN_OBJ_RX_SVC; o++) {
    if (CANMSG->Msg[o].MOSTAT_bit.MSGLST) {
      s_stats.rx_lost++;
      CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESMSGLST_Msk;
    }
    while (obj_rx_read(o, &f))
      rxq_push(&f);
    CANMSG->Msg[o].MOCTR = CANMSG_Msg_MOCTR_RESRXPND_Msk;
  }
}

/* --- публичный API -------------------------------------------------------- */

static void filters_config(uint8_t node_id) {
  obj_rx_config(CAN_OBJ_RX_RAW, DC_ID_RAWCOMMAND, DC_MASK_MSG_TYPE, CAN_RX_IRQ_LINE);
  obj_rx_config(CAN_OBJ_RX_RPM, DC_ID_RPMCOMMAND, DC_MASK_MSG_TYPE, CAN_RX_IRQ_LINE);
  if (node_id != 0u)
    obj_rx_config(CAN_OBJ_RX_SVC, ((uint32_t)node_id << 8) | 0x80u, DC_MASK_SVC_DEST,
                  CAN_RX_IRQ_LINE);
  else
    CANMSG->Msg[CAN_OBJ_RX_SVC].MOCTR = CANMSG_Msg_MOCTR_RESMSGVAL_Msk;
}

bool can_hw_init(uint16_t bitrate_kbps, uint8_t node_id, bool loopback) {
  uint32_t nbtr;
  if (!calc_nbtr(bitrate_kbps, &nbtr))
    return false;
  s_bitrate = bitrate_kbps;
  s_node_id = node_id;
  s_loopback = loopback;
  s_rx_head = s_rx_tail = 0u;
  memset((void *)&s_stats, 0, sizeof(s_stats));

  /* CAN0_RX = PB14, CAN0_TX = PB15 */
  RCU->HCLKCFG_bit.GPIOBEN = 1;
  RCU->HRSTCFG_bit.GPIOBEN = 1;
  GPIOB->DENSET = GPIO_DENSET_PIN14_Msk | GPIO_DENSET_PIN15_Msk;
  GPIOB->ALTFUNCSET = GPIO_ALTFUNCSET_PIN14_Msk | GPIO_ALTFUNCSET_PIN15_Msk;

  RCU->HCLKCFG_bit.CANEN = 1;
  RCU->HRSTCFG_bit.CANEN = 1;
  CAN->CLC_bit.DISR = 0;
  while (CAN->CLC_bit.DISS || CAN->PANCTR_bit.PANCMD) {
  }
  CAN->FDR = (0x1u << CAN_FDR_DM_Pos) | (0x3FFu << CAN_FDR_STEP_Pos); /* fCAN = HCLK */

  node_config(0u, nbtr, loopback);
  while (CAN->PANCTR_bit.BUSY) {
  }
  for (uint32_t o = CAN_OBJ_TX; o <= CAN_OBJ_RX_SVC; o++)
    obj_to_list(o, 1u);

  obj_tx_config(CAN_OBJ_TX);
  filters_config(node_id);

  NVIC_SetPriority(CAN2_IRQn, 3u); /* ниже ADC_SEQ0 (2): не мешает кольцу FOC */
  NVIC_ClearPendingIRQ(CAN2_IRQn);
  NVIC_EnableIRQ(CAN2_IRQn);
  return true;
}

void can_hw_set_node_id(uint8_t node_id) {
  s_node_id = node_id;
  NVIC_DisableIRQ(CAN2_IRQn);
  filters_config(node_id);
  NVIC_EnableIRQ(CAN2_IRQn);
}

bool can_hw_tx_busy(void) { return CANMSG->Msg[CAN_OBJ_TX].MOSTAT_bit.TXRQ != 0u; }

bool can_hw_tx(const can_frame_t *f) {
  if (can_hw_tx_busy())
    return false;
  obj_tx_send(CAN_OBJ_TX, f);
  s_stats.tx_frames++;
  return true;
}

void can_hw_tx_abort(void) {
  CANMSG->Msg[CAN_OBJ_TX].MOCTR = CANMSG_Msg_MOCTR_RESTXRQ_Msk;
  s_stats.tx_aborts++;
}

bool can_hw_rx_pop(can_frame_t *f) {
  if (s_rx_tail == s_rx_head)
    return false;
  f->id = s_rxq[s_rx_tail].id;
  f->dlc = s_rxq[s_rx_tail].dlc;
  for (uint32_t i = 0; i < 8u; i++)
    f->data[i] = s_rxq[s_rx_tail].data[i];
  s_rx_tail = (s_rx_tail + 1u) % RXQ_LEN;
  return true;
}

void can_hw_stats(can_stats_t *s) {
  s->rx_frames = s_stats.rx_frames;
  s->rx_lost = s_stats.rx_lost;
  s->tx_frames = s_stats.tx_frames;
  s->tx_aborts = s_stats.tx_aborts;
  s->bus_off_recoveries = s_stats.bus_off_recoveries;
  s->last_lec = (uint8_t)(CAN->Node[0].NSR & CAN_Node_NSR_LEC_Msk);
}

void can_hw_poll(void) {
  /* bus-off: узел сам ставит INIT; выйти из него = начать восстановление */
  if (CAN->Node[0].NSR & CAN_Node_NSR_BOFF_Msk) {
    CAN->Node[0].NCR &= ~CAN_Node_NCR_INIT_Msk;
    s_stats.bus_off_recoveries++;
  }
}

/* --- самотест на виртуальной шине --------------------------------------- */

static bool wait_rx1(can_frame_t *f) {
  for (uint32_t i = 0; i < 200000u; i++)
    if (obj_rx_read(CAN_OBJ_T1_RX, f))
      return true;
  return false;
}

static bool wait_rxq(can_frame_t *f) {
  for (uint32_t i = 0; i < 200000u; i++)
    if (can_hw_rx_pop(f))
      return true;
  return false;
}

static bool wait_tx1(void) {
  for (uint32_t i = 0; i < 200000u; i++)
    if (!CANMSG->Msg[CAN_OBJ_T1_TX].MOSTAT_bit.TXRQ)
      return true;
  return false;
}

uint32_t can_hw_selftest(uint16_t bitrate_kbps, uint8_t node_id) {
  uint32_t ok = 0u;
  uint32_t nbtr;
  if (node_id == 0u || !calc_nbtr(bitrate_kbps, &nbtr) ||
      !can_hw_init(bitrate_kbps, node_id, true))
    return 0u;

  /* узел CAN1 — «полётник» на виртуальной шине */
  node_config(1u, nbtr, true);
  obj_to_list(CAN_OBJ_T1_TX, 2u);
  obj_to_list(CAN_OBJ_T1_RX, 2u);
  obj_tx_config(CAN_OBJ_T1_TX);
  obj_rx_config(CAN_OBJ_T1_RX, 0u, 0u, 0u); /* приём всего (маска 0) */

  can_frame_t tx, rx;
  memset(&tx, 0, sizeof(tx));

  /* 0. CAN0 -> CAN1 */
  tx.id = 0x0A12345u;
  tx.dlc = 8u;
  for (uint32_t i = 0; i < 8u; i++)
    tx.data[i] = (uint8_t)(0x11u * (i + 1u));
  if (can_hw_tx(&tx) && wait_rx1(&rx) && rx.id == tx.id && rx.dlc == 8u &&
      memcmp(rx.data, tx.data, 8u) == 0)
    ok |= 1u << 0;

  /* 1. CAN1 -> CAN0: RawCommand (тип 1030, приоритет 1, источник узел 7) */
  tx.id = (1ul << 24) | DC_ID_RAWCOMMAND | 7u;
  tx.dlc = 3u;
  tx.data[0] = 0xA5u;
  tx.data[1] = 0x5Au;
  tx.data[2] = 0xC3u;
  obj_tx_send(CAN_OBJ_T1_TX, &tx);
  if (wait_tx1() && wait_rxq(&rx) && rx.id == tx.id && rx.dlc == 3u &&
      rx.data[0] == 0xA5u && rx.data[1] == 0x5Au && rx.data[2] == 0xC3u) {
    ok |= 1u << 1;
    ok |= 1u << 5; /* длина и данные сохранились */
  }

  /* 2. RPMCommand */
  tx.id = (2ul << 24) | DC_ID_RPMCOMMAND | 9u;
  tx.dlc = 8u;
  obj_tx_send(CAN_OBJ_T1_TX, &tx);
  if (wait_tx1() && wait_rxq(&rx) && rx.id == tx.id)
    ok |= 1u << 2;

  /* 3. сервис (GetNodeInfo = тип 1, запрос, получатель = наш узел) */
  tx.id = (24ul << 24) | (1ul << 16) | (1ul << 15) | ((uint32_t)node_id << 8) | 0x80u | 5u;
  tx.dlc = 1u;
  tx.data[0] = 0xC0u;
  obj_tx_send(CAN_OBJ_T1_TX, &tx);
  if (wait_tx1() && wait_rxq(&rx) && rx.id == tx.id)
    ok |= 1u << 3;

  /* 4. чужие кадры: другой тип сообщения и сервис для другого узла — не принимаются */
  can_frame_t junk;
  memset(&junk, 0, sizeof(junk));
  junk.id = (3ul << 24) | (5000ul << 8) | 4u;
  junk.dlc = 2u;
  obj_tx_send(CAN_OBJ_T1_TX, &junk);
  (void)wait_tx1();
  junk.id = (24ul << 24) | (1ul << 16) | ((uint32_t)(node_id ^ 0x15u) << 8) | 0x80u | 5u;
  obj_tx_send(CAN_OBJ_T1_TX, &junk);
  (void)wait_tx1();
  for (volatile uint32_t d = 0; d < 20000u; d++) {
  }
  if (!can_hw_rx_pop(&rx))
    ok |= 1u << 4;

  /* вернуть нормальный режим */
  CANMSG->Msg[CAN_OBJ_T1_TX].MOCTR = CANMSG_Msg_MOCTR_RESMSGVAL_Msk;
  CANMSG->Msg[CAN_OBJ_T1_RX].MOCTR = CANMSG_Msg_MOCTR_RESMSGVAL_Msk;
  CAN->Node[1].NCR = CAN_Node_NCR_CCE_Msk | CAN_Node_NCR_INIT_Msk;
  can_hw_init(s_bitrate, node_id, false);
  return ok;
}
