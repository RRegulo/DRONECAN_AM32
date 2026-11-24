/*
 * DroneCAN.c — узел DroneCAN ESC. Порт ref/bldc/libcanard/canard_driver.c (VESC,
 * GPLv3): те же сообщения, тот же смысл RawCommand (raw/8192 -> ток/RPM),
 * esc.Status, GetNodeInfo, param.GetSet/ExecuteOpcode, RestartNode.
 * Вместо ChibiOS-потока — dronecan_poll() из главного цикла; вместо
 * mc_interface_* — наше ядро (foc_closed_request / foc_closed_speed_request).
 */
#include "DroneCAN.h"

#include <math.h>
#include <string.h>

#include "CAN_Driver.h"
#include "FOC_Core.h"
#include "FOC_Params.h"
#include "FOC_Proto.h"
#include "canard.h"
#include "uavcan/equipment/esc/RPMCommand.h"
#include "uavcan/equipment/esc/RawCommand.h"
#include "uavcan/equipment/esc/Status.h"
#include "uavcan/protocol/GetNodeInfo.h"
#include "uavcan/protocol/NodeStatus.h"
#include "uavcan/protocol/RestartNode.h"
#include "uavcan/protocol/param/ExecuteOpcode.h"
#include "uavcan/protocol/param/GetSet.h"

#ifndef FOC_HOST_TEST
#include "K1921VK035.h"
#endif

#define POOL_BYTES 768u  /* 24 блока по 32 Б; пик в тесте обмена — 12 */
#define MSG_BYTES 128u
#define DYN_BYTES 192u
#define TX_STUCK_MS 20u

static CanardInstance s_ins;
static uint8_t s_pool[POOL_BYTES] __attribute__((aligned(8)));
static uint8_t s_msg[MSG_BYTES];
static uint8_t s_dyn[DYN_BYTES];
static bool s_enabled;
static uint8_t s_node_id;
static uint8_t s_esc_index;
static uint8_t s_raw_mode;
static float s_raw_rpm_max;
static uint16_t s_status_ms;
static uint16_t s_timeout_ms;

static bool s_can_active;
static uint32_t s_last_cmd_ms;
static uint32_t s_last_ns_ms;
static uint32_t s_last_status_ms;
static uint32_t s_last_cleanup_ms;
static uint32_t s_tx_stuck_since;
static bool s_tx_stuck_armed;
static uint32_t s_restart_at_ms;
static bool s_restart_pending;
static uint32_t s_start_ms;
static dronecan_stats_t s_st;

/* --- таблица параметров param.GetSet --------------------------------------- */
typedef struct {
  const char *name;
  uint8_t pid;
  uint8_t is_int;
  float min, max;
} TDcParam;

static const TDcParam s_dpar[] = {
    {"MOT_PP", FOC_PID_PP, 1, 1, 30},
    {"MOT_R_OHM", FOC_PID_R, 0, 0.001f, 10},
    {"MOT_L_H", FOC_PID_L, 0, 1e-6f, 10e-3f},
    {"MOT_LD_LQ_H", FOC_PID_LD_LQ, 0, -10e-3f, 10e-3f},
    {"MOT_LAMBDA_WB", FOC_PID_LAMBDA, 0, 10e-6f, 0.1f},
    {"MOT_KV", FOC_PID_KV, 0, 0, 100000},
    {"LIM_I_MAX_A", FOC_PID_IMAX, 0, 0.5f, 30},
    {"LIM_I_CMD_A", FOC_PID_ICONT, 0, 0.1f, 30},
    {"LIM_VBUS_MIN", FOC_PID_VBUS_MIN, 0, 3, 60},
    {"LIM_VBUS_MAX", FOC_PID_VBUS_MAX, 0, 4, 60},
    {"LIM_MAX_ERPM", FOC_PID_MAX_ERPM, 0, 100, 1e6f},
    {"SYS_FLAGS", FOC_PID_FLAGS, 1, 0, 255},
    {"VESC_DT_US", FOC_PID_DT_US, 0, 0, 3},
    {"VESC_CC_TC_US", FOC_PID_CC_TC_US, 0, 100, 20000},
    {"VESC_OBS_OFFSET", FOC_PID_OBS_OFFSET, 0, -3, 3},
    {"VESC_OL_RPM", FOC_PID_OL_RPM, 0, 100, 100000},
    {"VESC_SL_HYST", FOC_PID_SL_HYST, 0, 0, 5},
    {"VESC_SL_TIME", FOC_PID_SL_TIME, 0, 0, 5},
    {"VESC_SL_RAMP", FOC_PID_SL_RAMP, 0, 0, 5},
    {"SPD_KP", FOC_PID_S_KP, 0, 0, 100},
    {"SPD_KI", FOC_PID_S_KI, 0, 0, 100},
    {"SPD_KD", FOC_PID_S_KD, 0, 0, 100},
    {"SPD_KD_FILT", FOC_PID_S_KD_FILTER, 0, 0, 1},
    {"SPD_MIN_ERPM", FOC_PID_S_MIN_ERPM, 0, 0, 100000},
    {"SPD_RAMP_ERPMS", FOC_PID_S_RAMP_ERPMS, 0, 0, 1e6f},
    {"CAN_NODE_ID", FOC_PID_CAN_NODE_ID, 1, 0, 127},
    {"CAN_ESC_IDX", FOC_PID_CAN_ESC_INDEX, 1, 0, 19},
    {"CAN_RAW_MODE", FOC_PID_CAN_RAW_MODE, 1, 0, 3},
    {"CAN_RPM_MAX", FOC_PID_CAN_RAW_RPM_MAX, 0, 100, 1e6f},
    {"CAN_BITRATE", FOC_PID_CAN_BITRATE, 1, 100, 1000},
    {"CAN_STATUS_MS", FOC_PID_CAN_STATUS_MS, 1, 0, 5000},
    {"CAN_TIMEOUT_MS", FOC_PID_CAN_TIMEOUT_MS, 1, 0, 60000},
};
#define N_DPAR (sizeof(s_dpar) / sizeof(s_dpar[0]))

/* --- управление ------------------------------------------------------------- */

/* Командовать двигателем можно, если он свободен или уже ведётся по CAN: bench-режимы
 * UART (OPENLOOP/HOLD/детекция) не перебиваем. */
static bool can_may_control(void) {
  return s_can_active || g_foc_ctrl_state == FOC_CTRL_STOP;
}

static void cmd_release(void) {
  if (s_can_active)
    foc_ctrl_stop();
  s_can_active = false;
  if (g_foc_ctrl_state == FOC_CTRL_STOP && g_foc_fault_flags != 0u)
    (void)foc_fault_clear(); /* нулевая команда = «разоружение»: авария снимается, если условия ушли */
}

static void cmd_current(float raw) {
  if (raw < 0.0f)
    raw = 0.0f; /* реверса и торможения током нет (режимы 0/1) */
  float iq = raw * g_foc_i_cmd;
  if (iq < FOC_CC_MIN_CURRENT) {
    cmd_release();
    return;
  }
  if (foc_closed_request(iq))
    s_can_active = true;
}

static void cmd_erpm(float erpm) {
  if (erpm < g_s_pid_min_erpm) {
    cmd_release();
    return;
  }
  if (foc_closed_speed_request(erpm))
    s_can_active = true;
}

static void handle_raw_command(CanardRxTransfer *tr, uint32_t now_ms) {
  uavcan_equipment_esc_RawCommand cmd;
  memset(&cmd, 0, sizeof(cmd));
  uint8_t *tmp = s_dyn;
  if (uavcan_equipment_esc_RawCommand_decode(tr, tr->payload_len, &cmd, &tmp) < 0)
    return;
  if (cmd.cmd.len <= s_esc_index)
    return;
  float raw = (float)cmd.cmd.data[s_esc_index] / 8192.0f;
  if (!can_may_control())
    return;
  s_last_cmd_ms = now_ms;
  s_st.cmd_count++;
  switch (s_raw_mode) {
  case 0: /* CURRENT */
  case 1: /* CURRENT_NO_REV_BRAKE: тормоза током нет -> отрицательное = 0 */
    cmd_current(raw);
    break;
  case 3: /* RPM: raw * rpm_max (ERPM) */
    cmd_erpm(raw * s_raw_rpm_max);
    break;
  default: /* 2 DUTY: в ядре нет, команда игнорируется */
    break;
  }
}

static void handle_rpm_command(CanardRxTransfer *tr, uint32_t now_ms) {
  uavcan_equipment_esc_RPMCommand cmd;
  memset(&cmd, 0, sizeof(cmd));
  uint8_t *tmp = s_dyn;
  if (uavcan_equipment_esc_RPMCommand_decode(tr, tr->payload_len, &cmd, &tmp) < 0)
    return;
  if (cmd.rpm.len <= s_esc_index)
    return;
  if (!can_may_control())
    return;
  s_last_cmd_ms = now_ms;
  s_st.cmd_count++;
  /* VESC: значение уходит прямо в set_pid_speed (ERPM) */
  float rpm = (float)cmd.rpm.data[s_esc_index];
  cmd_erpm(rpm);
}

/* --- служебные сообщения ---------------------------------------------------- */

static void send_node_status(uint32_t now_ms) {
  static uint8_t tid;
  uavcan_protocol_NodeStatus ns;
  memset(&ns, 0, sizeof(ns));
  ns.uptime_sec = (now_ms - s_start_ms) / 1000u;
  ns.health = g_foc_fault_flags ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR
                                : UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  ns.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  ns.vendor_specific_status_code = g_foc_fault_flags;
  uavcan_protocol_NodeStatus_encode(&ns, s_msg);
  canardBroadcast(&s_ins, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID,
                  &tid, CANARD_TRANSFER_PRIORITY_LOW, s_msg, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
}

static void send_esc_status(void) {
  static uint8_t tid;
  uavcan_equipment_esc_Status st;
  memset(&st, 0, sizeof(st));
  st.error_count = g_foc_fault_flags;
  st.voltage = g_foc_udc;
  st.current = g_park.qe; /* ток момента, знаковый */
  st.temperature = NAN;   /* датчика температуры нет: «неизвестно» */
  st.rpm = (int32_t)g_observer.speed_rpm; /* механические об/мин, как у VESC */
  float pr = g_foc_i_cmd > 0.0f ? 100.0f * fabsf(g_park.qe) / g_foc_i_cmd : 0.0f;
  st.power_rating_pct = (uint8_t)(pr > 127.0f ? 127.0f : pr);
  st.esc_index = s_esc_index & 0x1Fu;
  uavcan_equipment_esc_Status_encode(&st, s_msg);
  canardBroadcast(&s_ins, UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE, UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                  &tid, CANARD_TRANSFER_PRIORITY_LOW, s_msg, UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE);
}

static void respond(CanardRxTransfer *tr, uint64_t sig, uint16_t id, uint16_t len) {
  uint8_t tid = tr->transfer_id;
  (void)canardRequestOrRespond(&s_ins, tr->source_node_id, sig, id, &tid, tr->priority,
                               CanardResponse, s_msg, len);
}

static void handle_get_node_info(CanardRxTransfer *tr, uint32_t now_ms) {
  uavcan_protocol_GetNodeInfoResponse pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.status.uptime_sec = (now_ms - s_start_ms) / 1000u;
  pkt.status.health = g_foc_fault_flags ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR
                                        : UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  pkt.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  pkt.software_version.major = DRONECAN_FW_MAJOR;
  pkt.software_version.minor = DRONECAN_FW_MINOR;
  pkt.hardware_version.major = 1;
  pkt.hardware_version.minor = 0;
  /* Уникального ID кристалла в РП нет: метка + node id (статические адреса) */
  memcpy(pkt.hardware_version.unique_id, "FOC1K1946VK035", 14);
  pkt.hardware_version.unique_id[14] = s_node_id;
  char name[] = DRONECAN_NODE_NAME;
  pkt.name.len = (uint8_t)(sizeof(name) - 1u);
  pkt.name.data = (uint8_t *)name;
  uint16_t total = (uint16_t)uavcan_protocol_GetNodeInfoResponse_encode(&pkt, s_msg);
  respond(tr, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE, UAVCAN_PROTOCOL_GETNODEINFO_ID, total);
}

static void handle_restart_node(CanardRxTransfer *tr, uint32_t now_ms) {
  uavcan_protocol_RestartNodeRequest req;
  memset(&req, 0, sizeof(req));
  uint8_t *tmp = s_dyn;
  bool ok = uavcan_protocol_RestartNodeRequest_decode(tr, tr->payload_len, &req, &tmp) >= 0 &&
            req.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER &&
            g_foc_ctrl_state == FOC_CTRL_STOP;
  uavcan_protocol_RestartNodeResponse resp;
  resp.ok = ok;
  uint16_t total = (uint16_t)uavcan_protocol_RestartNodeResponse_encode(&resp, s_msg);
  respond(tr, UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE, UAVCAN_PROTOCOL_RESTARTNODE_ID, total);
  if (ok) {
    s_restart_pending = true;
    s_restart_at_ms = now_ms + 100u; /* дать уйти ответу */
  }
}

static const TDcParam *dpar_by_name(const uint8_t *name, uint8_t len) {
  for (uint32_t i = 0; i < N_DPAR; i++)
    if (strlen(s_dpar[i].name) == len && memcmp(s_dpar[i].name, name, len) == 0)
      return &s_dpar[i];
  return NULL;
}

static void fill_value(uavcan_protocol_param_Value *v, const TDcParam *p, float x) {
  if (p->is_int) {
    v->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
    v->integer_value = (int64_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
  } else {
    v->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
    v->real_value = x;
  }
}

static void fill_numeric(uavcan_protocol_param_NumericValue *v, const TDcParam *p, float x) {
  if (p->is_int) {
    v->union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
    v->integer_value = (int64_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
  } else {
    v->union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
    v->real_value = x;
  }
}

static void handle_param_getset(CanardRxTransfer *tr) {
  uavcan_protocol_param_GetSetRequest req;
  memset(&req, 0, sizeof(req));
  uint8_t *tmp = s_dyn;
  if (uavcan_protocol_param_GetSetRequest_decode(tr, tr->payload_len, &req, &tmp) < 0)
    return;

  const TDcParam *p = NULL;
  if (req.name.len != 0u)
    p = dpar_by_name(req.name.data, req.name.len);
  else if (req.index < N_DPAR)
    p = &s_dpar[req.index];

  /* set: только по имени и только когда мотор свободен (как UART SET_PARAM) */
  if (p != NULL && req.name.len != 0u &&
      req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY && foc_param_idle()) {
    float x = 0.0f;
    bool have = true;
    if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
      x = (float)req.value.integer_value;
    else if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE)
      x = req.value.real_value;
    else if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE)
      x = req.value.boolean_value ? 1.0f : 0.0f;
    else
      have = false;
    if (have)
      (void)foc_param_set_f32(p->pid, x); /* вне диапазона — молча остаётся старое */
  }

  uavcan_protocol_param_GetSetResponse pkt;
  memset(&pkt, 0, sizeof(pkt));
  if (p != NULL) {
    fill_value(&pkt.value, p, foc_param_get_f32(p->pid));
    fill_value(&pkt.default_value, p, foc_param_default_f32(p->pid));
    fill_numeric(&pkt.min_value, p, p->min);
    fill_numeric(&pkt.max_value, p, p->max);
    pkt.name.len = (uint8_t)strlen(p->name);
    pkt.name.data = (uint8_t *)p->name;
  } /* иначе: пустое имя = конец перечисления */
  uint16_t total = (uint16_t)uavcan_protocol_param_GetSetResponse_encode(&pkt, s_msg);
  respond(tr, UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE, UAVCAN_PROTOCOL_PARAM_GETSET_ID, total);
}

static void handle_execute_opcode(CanardRxTransfer *tr) {
  uavcan_protocol_param_ExecuteOpcodeRequest req;
  memset(&req, 0, sizeof(req));
  uint8_t *tmp = s_dyn;
  if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(tr, tr->payload_len, &req, &tmp) < 0)
    return;
  bool ok = false;
  if (foc_param_idle()) {
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
      ok = foc_params_save();
    } else if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
      foc_params_defaults(&g_params);
      foc_params_apply();
      ok = foc_params_save();
    }
  }
  uavcan_protocol_param_ExecuteOpcodeResponse resp;
  memset(&resp, 0, sizeof(resp));
  resp.ok = ok;
  uint16_t total = (uint16_t)uavcan_protocol_param_ExecuteOpcodeResponse_encode(&resp, s_msg);
  respond(tr, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
          total);
}

/* --- libcanard callbacks ---------------------------------------------------- */

static uint32_t s_now_ms; /* для обработчиков (обновляется в dronecan_poll) */

static bool should_accept(const CanardInstance *ins, uint64_t *out_sig, uint16_t id,
                          CanardTransferType tt, uint8_t src) {
  (void)ins;
  (void)src;
  if (tt == CanardTransferTypeBroadcast) {
    if (id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
      *out_sig = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
      return true;
    }
    if (id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID) {
      *out_sig = UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_SIGNATURE;
      return true;
    }
    return false;
  }
  if (tt == CanardTransferTypeRequest) {
    switch (id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      *out_sig = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      *out_sig = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      *out_sig = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      *out_sig = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
      return true;
    default:
      break;
    }
  }
  return false;
}

static void on_reception(CanardInstance *ins, CanardRxTransfer *tr) {
  (void)ins;
  s_st.rx_transfers++;
  if (tr->transfer_type == CanardTransferTypeBroadcast) {
    if (tr->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
      handle_raw_command(tr, s_now_ms);
    else if (tr->data_type_id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID)
      handle_rpm_command(tr, s_now_ms);
  } else if (tr->transfer_type == CanardTransferTypeRequest) {
    switch (tr->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      handle_get_node_info(tr, s_now_ms);
      break;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      handle_restart_node(tr, s_now_ms);
      break;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      handle_param_getset(tr);
      break;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      handle_execute_opcode(tr);
      break;
    default:
      break;
    }
  }
}

/* --- публичный API ---------------------------------------------------------- */

bool dronecan_init(void) {
  memset(&s_st, 0, sizeof(s_st));
  s_enabled = false;
  s_node_id = g_params.s.can_node_id;
  if (s_node_id == 0u)
    return false; /* CAN выключен параметром */
  s_esc_index = g_params.s.can_esc_index;
  s_raw_mode = g_params.s.can_raw_mode;
  s_raw_rpm_max = g_params.s.can_raw_rpm_max;
  s_status_ms = g_params.s.can_status_ms;
  s_timeout_ms = g_params.s.can_timeout_ms;
  if (!can_hw_init(g_params.s.can_bitrate_kbps, s_node_id, false))
    return false;
  canardInit(&s_ins, s_pool, sizeof(s_pool), on_reception, should_accept, NULL);
  canardSetLocalNodeID(&s_ins, s_node_id);
  s_can_active = false;
  s_start_ms = s_last_ns_ms = s_last_status_ms = s_last_cleanup_ms = 0u;
  s_restart_pending = false;
  s_tx_stuck_armed = false;
  s_enabled = true;
  return true;
}

bool dronecan_enabled(void) { return s_enabled; }

void dronecan_stats(dronecan_stats_t *s) {
  *s = s_st;
  s->can_active = s_can_active ? 1u : 0u;
}

void dronecan_poll(uint32_t now_ms) {
  if (!s_enabled)
    return;
  s_now_ms = now_ms;
  can_hw_poll();

  /* приём (ограничено, чтобы не затянуть главный цикл) */
  can_frame_t f;
  for (uint32_t n = 0; n < 16u && can_hw_rx_pop(&f); n++) {
    CanardCANFrame cf;
    memset(&cf, 0, sizeof(cf));
    cf.id = f.id | CANARD_CAN_FRAME_EFF;
    cf.data_len = f.dlc;
    memcpy(cf.data, f.data, 8u);
    (void)canardHandleRxFrame(&s_ins, &cf, (uint64_t)now_ms * 1000u);
  }

  /* таймаут команд: потеряли полётник -> мотор отпускаем */
  if (s_can_active && s_timeout_ms != 0u && (now_ms - s_last_cmd_ms) > s_timeout_ms) {
    foc_ctrl_stop();
    s_can_active = false;
    s_st.timeouts++;
  }

  if ((now_ms - s_last_ns_ms) >= 1000u) {
    s_last_ns_ms = now_ms;
    send_node_status(now_ms);
  }
  if (s_status_ms != 0u && (now_ms - s_last_status_ms) >= s_status_ms) {
    s_last_status_ms = now_ms;
    send_esc_status();
  }
  if ((now_ms - s_last_cleanup_ms) >= 1000u) {
    s_last_cleanup_ms = now_ms;
    canardCleanupStaleTransfers(&s_ins, (uint64_t)now_ms * 1000u);
  }

  /* передача: кадры из очереди libcanard в объект TX */
  for (uint32_t n = 0; n < 8u; n++) {
    const CanardCANFrame *cf = canardPeekTxQueue(&s_ins);
    if (cf == NULL) {
      s_tx_stuck_armed = false;
      break;
    }
    can_frame_t tx;
    tx.id = cf->id & CANARD_CAN_EXT_ID_MASK;
    tx.dlc = cf->data_len;
    memcpy(tx.data, cf->data, 8u);
    if (can_hw_tx(&tx)) {
      canardPopTxQueue(&s_ins);
      s_st.tx_frames++;
      s_tx_stuck_armed = false;
    } else {
      if (!s_tx_stuck_armed) {
        s_tx_stuck_armed = true;
        s_tx_stuck_since = now_ms;
      } else if ((now_ms - s_tx_stuck_since) > TX_STUCK_MS) {
        can_hw_tx_abort(); /* нет ACK (пустая шина): кадр теряем, очередь не копится */
        canardPopTxQueue(&s_ins);
        s_st.tx_dropped++;
        s_tx_stuck_armed = false;
      }
      break;
    }
  }

  if (s_restart_pending && now_ms >= s_restart_at_ms) {
#ifndef FOC_HOST_TEST
    NVIC_SystemReset();
#endif
    s_restart_pending = false;
  }
}
