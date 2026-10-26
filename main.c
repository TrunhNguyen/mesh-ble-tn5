#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "boards.h"
#include "app_timer.h"
#include "custom_gpio_drivers.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

#include "bsp.h"
#include "bsp_btn_ble.h"

#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

#include "generic_level_server.h"
#include "scene_setup_server.h"
#include "model_config_file.h"
#include "generic_level_client.h"

#include "log.h"
#include "rtt_input.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_mesh_config_app.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "nrf_mesh_config_examples.h"
#include "app_level.h"
#include "app_dtt.h"
#include "ble_softdevice_support.h"
#include "nrf_delay.h"

#include "nrf_drv_wdt.h"

#include "nrf_drv_saadc.h"
#define MQ2_ANALOG_PIN NRF_SAADC_INPUT_AIN0

#include "st7789_drivers.h"
#include "nrf52_timer.h"
#include "nrf52_at24c32_64.h"
#include "nrf52_aht10.h"
#include "DHT.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

/* ===================== CAU HINH TN5 (chi sua o day) ===================== */
#define EXP_TN5_ENERGY                1

/* Bien the build: 0 = A (ngu sau), 1 = B/C (hoat dong binh thuong) */
#ifndef EXP_TN5_DEEPSLEEP
#define EXP_TN5_DEEPSLEEP             0
#endif
#define EXP_TN5_DS                    EXP_TN5_DEEPSLEEP

/* Vai tro: 0 = LPN (relay tat) | 1 = relay voi EXP_TN5_RELAY_RETX = 0, 2 hoac 5 */
#ifndef EXP_TN5_ROLE
#define EXP_TN5_ROLE                  1
#endif
#ifndef EXP_TN5_RELAY_RETX
#define EXP_TN5_RELAY_RETX            2
#endif
#if (EXP_TN5_ROLE == 0)
#define EXP_RELAY_EXPECT_EN           0
#define EXP_TN5_CFG_IDX               9
#else
#define EXP_RELAY_EXPECT_EN           1
#if   (EXP_TN5_RELAY_RETX == 0)
#define EXP_TN5_CFG_IDX               1
#elif (EXP_TN5_RELAY_RETX == 2)
#define EXP_TN5_CFG_IDX               9
#elif (EXP_TN5_RELAY_RETX == 5)
#define EXP_TN5_CFG_IDX               17
#else
#error "EXP_TN5_RELAY_RETX chi nhan 0, 2 hoac 5"
#endif
#endif
#define EXP_STOCK_IDX                 EXP_TN5_CFG_IDX

/* Xung TRIG: 1 cho bien the B (cat trang thai), 0 cho A va C (do dong TB, TRIG lam sai dong) */
#ifndef EXP_ENABLE_TRIG
#define EXP_ENABLE_TRIG               0
#endif

/* Chu ky nhiem vu */
#define EXP_TN5_AUTORUN               1
#define EXP_TN5_AUTORUN_DELAY_S       30
#define EXP_TN5_SENSE_PERIOD_S        5
#define EXP_TN5_MQ2_ON_S              20
#define EXP_TN5_MQ2_PERIOD_S          300
#define EXP_TN5_MQ2_SETTLE_S          15
#define EXP_TN5_LCD_SLEEP             1
#define EXP_TN5_DS_MESH_DISABLE       1
#ifndef MQ2_HEATER_ACTIVE_LEVEL
#define MQ2_HEATER_ACTIVE_LEVEL       1
#endif
/* MQ2_HEATER_EN_PIN: KHONG bat buoc. Neu khai bao (-DMQ2_HEATER_EN_PIN=n) thi firmware duty-cycle soi dot;
 * neu khong thi soi dot chay lien tuc nhu cac TN truoc. */

/* Co dinh cho TN5 */
#define EXP_ID                        5
#define FW_GIT_HASH                   "tn5_v1"
#define EXP_NUM_MSGS                  65535
#define EXP_HOP_WHITELIST             0
#define EXP_DEBUG_RX_MAC              0
#define EXP_RTT_KEYS                  0
#define EXP_LIVE_LOG                  0
#define EXP_TN4_LOCK_STOCK            1
/* ======================================================================== */

#ifndef EXP_ID
#define EXP_ID                    4
#endif
#ifndef FW_GIT_HASH
#define FW_GIT_HASH               "tn4_v1"
#endif
#ifndef EXP_NUM_MSGS
#define EXP_NUM_MSGS              1000
#endif
#ifndef EXP_PERIOD_MS
#define EXP_PERIOD_MS             1000
#endif
#ifndef EXP_DEFAULT_TTL_INIT
#define EXP_DEFAULT_TTL_INIT      4
#endif
#ifndef EXP_TX_POWER_DBM
#define EXP_TX_POWER_DBM          0
#endif
#ifndef EXP_SEND_UNACK
#define EXP_SEND_UNACK            1
#endif
#ifndef EXP_READ_CFG
#define EXP_READ_CFG              1
#endif
#ifndef EXP_ENABLE_TRIG
#define EXP_ENABLE_TRIG           1
#endif
#ifndef EXP_LIVE_LOG
#define EXP_LIVE_LOG              0
#endif
#ifndef EXP_ENABLE_ALARM_BUTTON
#define EXP_ENABLE_ALARM_BUTTON   0
#endif
#ifndef EXP_HOPS
#define EXP_HOPS                  3
#endif
#ifndef EXP_DEBUG_RX_MAC
#define EXP_DEBUG_RX_MAC          0
#endif
#ifndef EXP_MAX_TX_FAIL
#define EXP_MAX_TX_FAIL           30
#endif
#ifndef EXP_HALT_ON_FAULT
#define EXP_HALT_ON_FAULT         1
#endif
#ifndef EXP_STRICT_CFG
#define EXP_STRICT_CFG            1
#endif
#ifndef EXP_RTT_KEYS
#define EXP_RTT_KEYS              1
#endif
#ifndef EXP_TRACE_STEPS
#define EXP_TRACE_STEPS           0
#endif
#ifndef EXP_HOP_WHITELIST
#define EXP_HOP_WHITELIST         1
#endif
#ifndef EXP_CACHE_COUNT_BACK_ECHO
#define EXP_CACHE_COUNT_BACK_ECHO 1
#endif
#ifndef EXP_HOPS_RUNTIME
#define EXP_HOPS_RUNTIME          0
#endif
#ifndef EXP_TN3_PRESET_RUNTIME
#define EXP_TN3_PRESET_RUNTIME    1
#endif
/* TN4: cau hinh stock = idx 9 trong EXP_TN3_TABLE = {relay_retx 2, net_tx 2, ttl 4}.
 * EXP_TN4_LOCK_STOCK=1: khoa cung idx 9 (khong cho doi bang nut/RTT) de 36 run
 * khong the vo tinh chay sai cau hinh, va de khop duoc voi TN2 3 chang. */
#ifndef EXP_TN4_LOCK_STOCK
#define EXP_TN4_LOCK_STOCK        1
#endif
#ifndef EXP_STOCK_IDX
#define EXP_STOCK_IDX             9
#endif
#ifndef EXP_TN3_START_IDX
#define EXP_TN3_START_IDX         EXP_STOCK_IDX
#endif

#if (EXP_PERIOD_MS != 1000)
#warning "Chu ky phat thuc te co dinh 1000 ms"
#endif
#if (EXP_HOPS < 1) || (EXP_HOPS > 4)
#error "EXP_HOPS phai nam trong 1..4"
#endif

#define EXP_PAYLOAD_LEN           5

#define EXP_STEP(s)               ((void)0)

#include "mesh_opt_core.h"

#ifndef TRIG_PIN
#define TRIG_PIN                  11
#endif
#ifndef TRIG_WIDTH_US
#define TRIG_WIDTH_US             200
#endif
#ifndef TRIG_GAP_US
#define TRIG_GAP_US               200
#endif

#define EXP_TRIG_TX               1
#define EXP_TRIG_FWD              2
#define EXP_TRIG_RX               3
/* TN5: marker trang thai (chi co tac dung khi EXP_ENABLE_TRIG=1, bien the B) */
#define EXP_TRIG_SENSE            4   /* bat dau chu ky doc cam bien AHT10 (+MQ-2) */
#define EXP_TRIG_HEAT_ON          5   /* soi dot MQ-2 bat */
#define EXP_TRIG_HEAT_OFF         6   /* soi dot MQ-2 tat */

static volatile uint32_t m_trig_cnt[8];

void exp_trig_begin(void)
{
#if EXP_ENABLE_TRIG
    nrf_gpio_pin_set(TRIG_PIN);
#endif
}

void exp_trig_end(uint8_t pulses)
{
#if EXP_ENABLE_TRIG
    nrf_delay_us(TRIG_WIDTH_US);
    nrf_gpio_pin_clear(TRIG_PIN);
    for (uint8_t i = 1; i < pulses; i++)
    {
        nrf_delay_us(TRIG_GAP_US);
        nrf_gpio_pin_set(TRIG_PIN);
        nrf_delay_us(TRIG_WIDTH_US);
        nrf_gpio_pin_clear(TRIG_PIN);
    }
    if (pulses < 8)
    {
        m_trig_cnt[pulses]++;
    }
#else
    (void)pulses;
#endif
}

static inline void exp_trig_marker(uint8_t pulses)
{
    exp_trig_begin();
    exp_trig_end(pulses);
}

typedef struct {
    uint8_t relay_retx;
    uint8_t net_tx;
    uint8_t ttl;
} exp_tn3_cfg_t;

#define EXP_TN3_NUM_CFGS          24

static const exp_tn3_cfg_t EXP_TN3_TABLE[EXP_TN3_NUM_CFGS] = {
    {0, 1, 4}, {0, 2, 4}, {0, 3, 4}, {0, 5, 4},
    {1, 1, 4}, {1, 2, 4}, {1, 3, 4}, {1, 5, 4},
    {2, 1, 4}, {2, 2, 4}, {2, 3, 4}, {2, 5, 4},
    {3, 1, 4}, {3, 2, 4}, {3, 3, 4}, {3, 5, 4},
    {5, 1, 4}, {5, 2, 4}, {5, 3, 4}, {5, 5, 4},
    {2, 2, 1}, {2, 2, 2}, {2, 2, 3}, {2, 2, 5}
};

#define EXP_NOINIT_MAGIC          0x544E3336UL
#define EXP_GPREG_VALID_BIT       0x80u
#define EXP_GPREG_MIDRST_BIT      0x40u
#define EXP_GPREG_INRUN_BIT       0x20u
#define EXP_GPREG_IDX_MASK        0x1Fu

typedef struct {
    uint32_t magic;
    uint8_t  cfg_idx;
    uint8_t  in_run;
    uint8_t  mid_reset_flag;
    uint8_t  checksum;
} exp_noinit_t;

static __attribute__((section(".non_init"))) exp_noinit_t m_noinit;

static volatile uint8_t  m_tn3_cfg_idx      = EXP_TN3_START_IDX;
static volatile int8_t   m_req_tn3_target   = -1;
static volatile bool     m_cfg_match_ok     = false;
static volatile bool     m_boot_preset_done = false;
static volatile uint8_t  m_mid_run_reset    = 0;
static volatile uint16_t m_target_msgs      = EXP_NUM_MSGS;

static uint8_t exp_noinit_calc_csum(const exp_noinit_t * p)
{
    return (uint8_t)(0xA5u ^ p->cfg_idx ^ (p->in_run << 1) ^ (p->mid_reset_flag << 2));
}

#if defined(POWER_GPREGRET2_GPREGRET_Msk)
#define EXP_HAVE_GPREGRET2 1
#else
#define EXP_HAVE_GPREGRET2 0
#endif

static void exp_gpregret2_write(uint8_t val)
{
#if EXP_HAVE_GPREGRET2
    uint8_t sd_en = 0;
    (void)sd_softdevice_is_enabled(&sd_en);
    if (sd_en)
    {
        (void)sd_power_gpregret_clr(1, 0xFFu);
        (void)sd_power_gpregret_set(1, val);
    }
    else
    {
        NRF_POWER->GPREGRET2 = (uint32_t)val;
    }
#else
    (void)val;
#endif
}

static uint8_t exp_gpregret2_read(void)
{
#if EXP_HAVE_GPREGRET2
    uint8_t sd_en = 0;
    (void)sd_softdevice_is_enabled(&sd_en);
    if (sd_en)
    {
        uint32_t v = 0;
        if (sd_power_gpregret_get(1, &v) == NRF_SUCCESS)
        {
            return (uint8_t)(v & 0xFFu);
        }
    }
    return (uint8_t)(NRF_POWER->GPREGRET2 & 0xFFu);
#else
    return 0u;
#endif
}

static void exp_noinit_save(uint8_t cfg_idx, uint8_t in_run, uint8_t mid_rst)
{
    m_noinit.magic          = EXP_NOINIT_MAGIC;
    m_noinit.cfg_idx        = cfg_idx;
    m_noinit.in_run         = in_run;
    m_noinit.mid_reset_flag = mid_rst;
    m_noinit.checksum       = exp_noinit_calc_csum(&m_noinit);

    uint8_t reg_val = EXP_GPREG_VALID_BIT |
                      (cfg_idx & EXP_GPREG_IDX_MASK) |
                      (in_run ? EXP_GPREG_INRUN_BIT : 0u) |
                      (mid_rst ? EXP_GPREG_MIDRST_BIT : 0u);
    exp_gpregret2_write(reg_val);
}

static void exp_noinit_check_on_boot(void)
{
    uint8_t reg_val = exp_gpregret2_read();
    if ((reg_val & EXP_GPREG_VALID_BIT) != 0u)
    {
        uint8_t idx = reg_val & EXP_GPREG_IDX_MASK;
        if (idx < EXP_TN3_NUM_CFGS)
        {
            m_tn3_cfg_idx = idx;
            if ((reg_val & (EXP_GPREG_INRUN_BIT | EXP_GPREG_MIDRST_BIT)) != 0u)
            {
                m_mid_run_reset = 1;
            }
            exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);
            return;
        }
    }

    if (m_noinit.magic == EXP_NOINIT_MAGIC &&
        m_noinit.checksum == exp_noinit_calc_csum(&m_noinit) &&
        m_noinit.cfg_idx < EXP_TN3_NUM_CFGS)
    {
        m_tn3_cfg_idx = m_noinit.cfg_idx;
        if (m_noinit.in_run != 0 || m_noinit.mid_reset_flag != 0)
        {
            m_mid_run_reset = 1;
        }
        exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);
    }
    else
    {
        m_tn3_cfg_idx   = EXP_TN3_START_IDX;
        m_mid_run_reset = 0;
        exp_noinit_save(m_tn3_cfg_idx, 0, 0);
    }
}

typedef struct {
    uint8_t addr[BLE_GAP_ADDR_LEN];
} exp_node_mac_t;

static const exp_node_mac_t EXP_TOPOLOGY_CHAIN[5] = {
    { {0x4D, 0xEA, 0x4F, 0x49, 0x8F, 0xF4} },
    { {0xA6, 0x63, 0x52, 0xE2, 0x7B, 0xCB} },
    { {0x16, 0x19, 0x71, 0x80, 0x5B, 0xC9} },
    { {0xD0, 0x26, 0xE8, 0x7D, 0x07, 0xD2} },
    { {0x4A, 0x5A, 0x32, 0xA2, 0x18, 0xF8} },
};

static int8_t m_my_chain_index = -1;
static ble_gap_addr_t m_my_ble_addr;

void exp_chain_mac_init(void)
{
    (void)sd_ble_gap_addr_get(&m_my_ble_addr);

    SEGGER_RTT_printf(0, "# [NODE_ADDR] MAC (UI):  %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                      m_my_ble_addr.addr[5], m_my_ble_addr.addr[4], m_my_ble_addr.addr[3],
                      m_my_ble_addr.addr[2], m_my_ble_addr.addr[1], m_my_ble_addr.addr[0]);

    SEGGER_RTT_printf(0, "# [NODE_ADDR] ARRAY C:   { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X }\r\n",
                      m_my_ble_addr.addr[0], m_my_ble_addr.addr[1], m_my_ble_addr.addr[2],
                      m_my_ble_addr.addr[3], m_my_ble_addr.addr[4], m_my_ble_addr.addr[5]);

    m_my_chain_index = -1;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (memcmp(m_my_ble_addr.addr, EXP_TOPOLOGY_CHAIN[i].addr, BLE_GAP_ADDR_LEN) == 0)
        {
            m_my_chain_index = (int8_t)i;
            break;
        }
    }

}

#define EXP_GW_INDEX              4

volatile uint8_t g_exp_hops = EXP_HOPS;

static inline bool exp_is_dest(void)
{
    return true;
}

extern uint32_t m_secondCounter;

static uint32_t m_rtc_last   = 0;
static uint64_t m_rtc_acc    = 0;
static uint64_t m_rtc_t0     = 0;

static inline uint32_t get_local_time_us(void)
{
    uint32_t now, diff;
    uint64_t acc;

    CRITICAL_REGION_ENTER();
    now  = app_timer_cnt_get() & 0x00FFFFFF;
    diff = (now - m_rtc_last) & 0x00FFFFFF;
    m_rtc_last = now;
    m_rtc_acc += diff;
    acc = m_rtc_acc - m_rtc_t0;
    CRITICAL_REGION_EXIT();

    return (uint32_t)((acc * 1000000ULL) / 32768ULL);
}

static void exp_time_reset(void)
{
    CRITICAL_REGION_ENTER();
    uint32_t now = app_timer_cnt_get() & 0x00FFFFFF;
    m_rtc_acc += (now - m_rtc_last) & 0x00FFFFFF;
    m_rtc_last = now;
    m_rtc_t0   = m_rtc_acc;
    CRITICAL_REGION_EXIT();
}

enum signalStatus
{
    _normalStatus = 1,
    _fireAlarmStatus = 3,
};

static uint32_t m_present_level;
uint32_t timerBCounter;
uint32_t refreshDisplayCounter = 1;
bool _getResetProvisionKey = false;

volatile uint16_t g_last_mesh_src_addr = 0;
volatile int8_t   g_last_mesh_rssi     = 0;
volatile uint8_t  g_last_mesh_ttl      = 0;

bool clientProvisioned = false;
bool clientProvisionChanged = false;

static void trigger_egu30_event(void);

bool aht10Available;
float _clientTemperature;
float _clientHumidity;

bool mq2Available = true;
uint16_t _clientMQ2Value;

bool clearAllDisplayNodes;

#define UART_TX_BUF_SIZE        256
#define UART_RX_BUF_SIZE        256

void uart_error_handle(app_uart_evt_t * p_event)
{
    (void)p_event;
}

dsm_local_unicast_address_t node_address;

static volatile bool m_uart_ready = false;

void uart_puts(const char *_ch)
{
    if (!m_uart_ready)
    {
        return;
    }
    while (*_ch)
    {
        if (app_uart_put(*_ch) != NRF_SUCCESS)
        {
            break;
        }
        _ch++;
    }
}

void uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_PIN_DISCONNECTED,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = UART_PIN_DISCONNECTED,
        .cts_pin_no   = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
    #if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
    #else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
    #endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
    m_uart_ready = true;
}

#define LOG_QUEUE_SIZE        16
#define LOG_MSG_MAX_LEN       128

static char m_log_queue[LOG_QUEUE_SIZE][LOG_MSG_MAX_LEN];
static volatile uint8_t m_log_head = 0;
static volatile uint8_t m_log_tail = 0;
static volatile bool m_logging_enabled = true;
bool periodic_message_allowed = false;

static void log_uart_mesh_event(const char* event, uint16_t src, uint16_t dst,
                                uint32_t seq, uint8_t ttl_tx, uint8_t ttl_rx,
                                int8_t rssi, const char* traffic_class, uint8_t len)
{
    (void)event; (void)src; (void)dst; (void)seq; (void)ttl_tx; (void)ttl_rx;
    (void)rssi; (void)traffic_class; (void)len;
    return;
    if (!m_logging_enabled)
    {
        return;
    }
    uint32_t t_us = get_local_time_us();
    uint16_t my_id = node_address.address_start & 0xFFFF;

    uint8_t next_head = (m_log_head + 1) % LOG_QUEUE_SIZE;

    if (next_head != m_log_tail)
    {
        snprintf(m_log_queue[m_log_head], LOG_MSG_MAX_LEN,
                 "%u,0x%04X,%s,0x%04X,0x%04X,%u,%u,%u,%d,%s,%u\r\n",
                 t_us, my_id, event, src, dst, seq, ttl_tx, ttl_rx, rssi, traffic_class, len);

        SEGGER_RTT_WriteString(0, m_log_queue[m_log_head]);
        m_log_head = next_head;
    }
}

static void log_queue_flush(void)
{
    while (m_log_tail != m_log_head)
    {
        uart_puts(m_log_queue[m_log_tail]);
        m_log_tail = (m_log_tail + 1) % LOG_QUEUE_SIZE;
    }
}

extern nrfx_wdt_channel_id m_channel_id;

#define MAX_OFFLINE_RECORDS  1100

enum {
    EXP_EVT_TX    = 0,
    EXP_EVT_RX    = 1,
    EXP_EVT_RELAY = 2
};

#define PACK_TTL_EVT(ttl, evt)   ((uint8_t)((((evt) & 0x03u) << 6) | ((ttl) & 0x3Fu)))
#define UNPACK_TTL(ttl_evt)      ((uint8_t)((ttl_evt) & 0x3Fu))
#define UNPACK_EVT(ttl_evt)      ((uint8_t)(((ttl_evt) >> 6) & 0x03u))

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint16_t seq_alarm;
    uint16_t src;
    uint8_t  ttl_evt;
    int8_t   rssi;
    uint8_t  drop_cache;
} offline_log_t;

static offline_log_t m_offline_logs[MAX_OFFLINE_RECORDS];
static volatile uint16_t m_offline_count    = 0;
static volatile uint16_t m_offline_dropped  = 0;
static volatile bool     m_dump_requested   = false;
static volatile bool     m_status_requested = false;
volatile uint8_t         g_display_src      = 0;

static volatile uint32_t m_drop_cache_cnt   = 0;
static volatile uint32_t m_relay_ok_cnt     = 0;
static volatile uint32_t m_rx_unique_cnt    = 0;

static volatile bool     m_req_toggle       = false;
static volatile bool     m_run_done         = false;
static volatile bool     m_run_aborted      = false;
static volatile bool     m_role_tx          = false;
static volatile uint32_t m_tx_fail_total    = 0;
static volatile uint32_t m_tx_fail_consec   = 0;
static volatile uint32_t m_last_err         = 0;
static uint16_t          m_tx_dst           = 0xFFFF;
static uint32_t          g_reset_reason     = 0;

static uint8_t  m_cfg_ttl_pub      = 0xFF;
static uint8_t  m_cfg_rtx          = 0xFF;
static uint8_t  m_cfg_txcnt        = 0xFF;
static uint16_t m_cfg_txint_ms     = 0;
static int8_t   m_cfg_dbm          = 127;
static uint8_t  m_cfg_ttl_eff      = EXP_DEFAULT_TTL_INIT;
static uint8_t  m_cfg_relay_tx     = 0xFF;
static uint8_t  m_cfg_relay_en     = 0xFF;
static uint16_t m_cfg_relay_int_ms = 0;
static int      m_cfg_srv_pub      = -1;
static int      m_cfg_cli_pub      = -1;
static char     m_cfg_buf[768];
static char     m_line_buf[128];

/* TN4: nhan nhieu Wi-Fi, khai bao luc xa log bang RTT: 'wf=CH,LOAD' va 'run=K'.
 * 0xFF = chua khai bao. Tu xoa sau khi xa log thanh cong de khong bi nhan nham run cu. */
static volatile uint8_t  m_wifi_ch   = 0xFF;
static volatile uint16_t m_wifi_mbps = 0;
static volatile uint8_t  m_run_k     = 0;

static bool rtt_write_line_reliable(const char *p, unsigned len)
{
    uint32_t waited_ms = 0;
    while (SEGGER_RTT_Write(0, p, len) != len)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);
        nrf_delay_ms(1);
        if (++waited_ms >= 3000)
        {
            return false;
        }
    }
    return true;
}

static void offline_log_store_tx(uint16_t seq, uint8_t ttl, bool is_alarm, uint32_t t_us)
{
    if (!m_logging_enabled)
    {
        return;
    }
    if (m_offline_count >= MAX_OFFLINE_RECORDS)
    {
        m_offline_dropped++;
        return;
    }

    uint16_t idx = m_offline_count++;
    m_offline_logs[idx].t_us       = t_us;
    m_offline_logs[idx].seq_alarm  = ((is_alarm ? 1 : 0) << 15) | (seq & 0x7FFF);
    m_offline_logs[idx].src        = (uint16_t)(node_address.address_start & 0xFFFF);
    m_offline_logs[idx].ttl_evt    = PACK_TTL_EVT(ttl, EXP_EVT_TX);
    m_offline_logs[idx].rssi       = 0;
    m_offline_logs[idx].drop_cache = 0;
}

void offline_log_push_tx(uint16_t seq, uint8_t ttl, bool is_alarm)
{
    offline_log_store_tx(seq, ttl, is_alarm, get_local_time_us());
}

void offline_log_push_relay(uint16_t src,
                            uint16_t dst,
                            uint32_t net_seq,
                            uint8_t  ttl_tx,
                            uint8_t  ttl_rx,
                            int8_t   rssi)
{
    (void)dst;
    (void)ttl_tx;
    if (!m_logging_enabled)
    {
        return;
    }

    const uint32_t t_us = get_local_time_us();
    uint16_t seq_15 = (uint16_t)(net_seq & 0x7FFF);

    uint16_t clean_src = src;
    if (clean_src % 2 == 0 && clean_src > 1)
    {
        clean_src -= 1;
    }

    m_relay_ok_cnt++;

    if (m_offline_count < MAX_OFFLINE_RECORDS)
    {
        uint16_t idx = m_offline_count++;
        m_offline_logs[idx].t_us       = t_us;
        m_offline_logs[idx].seq_alarm  = seq_15;
        m_offline_logs[idx].src        = clean_src;
        m_offline_logs[idx].ttl_evt    = PACK_TTL_EVT(ttl_rx, EXP_EVT_RELAY);
        m_offline_logs[idx].rssi       = rssi;
        m_offline_logs[idx].drop_cache = 0;
    }
    else
    {
        m_offline_dropped++;
    }

    log_uart_mesh_event("RELAY", clean_src, dst,
                        seq_15, ttl_tx, ttl_rx, rssi,
                        "TELEMETRY", EXP_PAYLOAD_LEN);
}

void offline_log_on_drop_cache(uint16_t src,
                               uint32_t net_seq,
                               uint8_t  ttl_rx,
                               int8_t   rssi)
{
    if (!m_logging_enabled)
    {
        return;
    }

    uint16_t own = (uint16_t)(node_address.address_start & 0xFFFF);
    if (src == own || src == (uint16_t)(own + 1))
    {
        return;
    }

    m_drop_cache_cnt++;

    uint16_t seq_15 = (uint16_t)(net_seq & 0x7FFF);
    if (m_offline_count > 0)
    {
        uint16_t start = m_offline_count;
        uint16_t limit = (start > 16) ? (start - 16) : 0;
        for (uint16_t i = start; i > limit; i--)
        {
            uint16_t idx = i - 1;
            if ((m_offline_logs[idx].seq_alarm & 0x7FFF) == seq_15)
            {
                if (m_offline_logs[idx].drop_cache < 255)
                {
                    m_offline_logs[idx].drop_cache++;
                }
                goto log_live_drop;
            }
        }
        if (m_offline_logs[m_offline_count - 1].drop_cache < 255)
        {
            m_offline_logs[m_offline_count - 1].drop_cache++;
        }
    }

log_live_drop:
    log_uart_mesh_event("DROP_CACHE", src, own,
                        seq_15, 0, ttl_rx, rssi,
                        "DUPLICATE", EXP_PAYLOAD_LEN);
}

void offline_log_push(uint16_t src, uint8_t ttl, int8_t rssi, uint16_t payload)
{
    if (!m_logging_enabled)
    {
        return;
    }

    uint16_t own = (uint16_t)(node_address.address_start & 0xFFFF);
    if (src == own || src == (uint16_t)(own + 1))
    {
        return;
    }

    const bool emit = exp_is_dest();
    if (emit)
    {
        exp_trig_begin();
    }
    const uint32_t t_us = get_local_time_us();

    uint16_t raw_seq     = payload & 0x7FFF;
    uint8_t  traffic_bit = (payload >> 15) & 0x01;

    uint16_t clean_src = src;
    if (clean_src % 2 == 0 && clean_src > 1)
    {
        clean_src -= 1;
    }
    g_display_src = (uint8_t)(clean_src & 0xFF);

    m_rx_unique_cnt++;

    if (emit)
    {
        if (m_offline_count < MAX_OFFLINE_RECORDS)
        {
            uint16_t idx = m_offline_count++;
            m_offline_logs[idx].t_us       = t_us;
            m_offline_logs[idx].seq_alarm  = (traffic_bit << 15) | raw_seq;
            m_offline_logs[idx].src        = clean_src;
            m_offline_logs[idx].ttl_evt    = PACK_TTL_EVT(ttl, EXP_EVT_RX);
            m_offline_logs[idx].rssi       = rssi;
            m_offline_logs[idx].drop_cache = 0;
        }
        else
        {
            m_offline_dropped++;
        }
    }

    log_uart_mesh_event("RX", clean_src, node_address.address_start,
                        raw_seq, 0, ttl, rssi,
                        (traffic_bit ? "ALARM" : "TELEMETRY"), EXP_PAYLOAD_LEN);

    if (emit)
    {
        exp_trig_end(EXP_TRIG_RX);
    }
}

static void log_config_init(void)
{
    SEGGER_RTT_Init();
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_DEFAULT_BACKENDS_INIT();
    }
    SEGGER_RTT_WriteString(0, "\r\n\r\n====== RTT DA KET NOI THANH CONG (" FW_GIT_HASH ") ======\r\n");
    SEGGER_RTT_WriteString(0, "# Lenh RTT TN4: 'wf=CH,LOAD'+Enter (CH=0/1/6/11, LOAD=Mb/s do duoc), 'run=K'+Enter, 'n=10..1100'+Enter, '2' (Run), '3' (Dump), '4' (Clear), 's' (Status). Cau hinh khoa stock idx=9.\r\n\r\n");
}

void mq2_saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(MQ2_ANALOG_PIN);
    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

uint16_t read_mq2_sensor(void)
{
    nrf_saadc_value_t val = 0;
    /* TN5: SAADC chi bat trong luc lay mau, doc xong tat han (khong de SAADC enable luc ngu) */
    mq2_saadc_init();
    nrf_drv_saadc_sample_convert(0, &val);
    nrf_drv_saadc_uninit();
    if (val < 0) val = 0;
    return (uint16_t)val;
}

/* Soi dot MQ-2 ~800 mW neu chay lien tuc. Chi dieu khien duoc khi board co chan GPIO bat/tat no. */
#ifdef MQ2_HEATER_EN_PIN
static void mq2_heater_set(bool on)
{
    nrf_gpio_pin_write(MQ2_HEATER_EN_PIN, (on == (MQ2_HEATER_ACTIVE_LEVEL != 0)) ? 1 : 0);
}

static void mq2_heater_init(void)
{
    nrf_gpio_cfg_output(MQ2_HEATER_EN_PIN);
    mq2_heater_set(false);
}
#define EXP_TN5_HEATER_CTRL 1
#else
static void mq2_heater_set(bool on) { (void)on; }
static void mq2_heater_init(void) { }
#define EXP_TN5_HEATER_CTRL 0
#endif

#define APP_LEVEL_ELEMENT_INDEX       (0)
#define APP_FORCE_SEGMENTATION        (false)
#define APP_MIC_SIZE                  (NRF_MESH_TRANSMIC_SIZE_SMALL)
#define CLIENT_MODEL_INSTANCE_COUNT   (1)
#define APP_LEVEL_DELAY_MS            (100)
#define APP_LEVEL_TRANSITION_TIME_MS  (10)

static void app_generic_level_client_status_cb(const generic_level_client_t * p_self, const access_message_rx_meta_t * p_meta, const generic_level_status_params_t * p_in);
static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status);
static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self);

static generic_level_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];

const generic_level_client_callbacks_t client_cbs =
{
    .level_status_cb = app_generic_level_client_status_cb,
    .ack_transaction_status_cb = app_gen_level_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_level_client_publish_interval_cb
};

static void app_generic_level_client_status_cb(const generic_level_client_t * p_self, const access_message_rx_meta_t * p_meta, const generic_level_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Level server: 0x%04x, Present Level: %d, Target Level: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_level, p_in->target_level, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Level server: 0x%04x, Present Level: %d\n", p_meta->src.value, p_in->present_level);
    }
}

static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;
        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

static volatile uint16_t m_tx_packet_count = 0;
static uint32_t m_global_seq = 0;

static void publish_present_alarm_status(bool _statusSending)
{
    if (!_statusSending && m_tx_packet_count >= m_target_msgs)
    {
        periodic_message_allowed = false;
        m_run_done = true;
        return;
    }

    uint8_t client = 0;
    static generic_level_set_params_t set_params = {0};

    const char *p_class = "TELEMETRY";
    uint16_t traffic_bit = 0;

    if (_statusSending)
    {
        traffic_bit = 1;
        p_class = "ALARM";
    }

    static uint8_t m_tid_counter = 0;
    uint32_t next_seq = m_global_seq + 1;
    uint16_t seq_15bit = (uint16_t)(next_seq & 0x7FFF);

    uint16_t payload = (traffic_bit << 15) | seq_15bit;

    set_params.level = (int16_t)payload;
    set_params.tid   = m_tid_counter++;

    if (m_tx_packet_count < 3) { EXP_STEP("set_unack"); }

    exp_trig_begin();
    const uint32_t t_tx_us = get_local_time_us();

    uint32_t status = generic_level_client_set_unack(&m_clients[client], &set_params, NULL, 0);

    exp_trig_end(EXP_TRIG_TX);

    if (status == NRF_SUCCESS)
    {
        m_global_seq = next_seq;
        m_tx_fail_consec = 0;

        if (!_statusSending)
        {
            m_tx_packet_count++;
        }

        uint8_t current_ttl = 0xFF;
        (void)access_model_publish_ttl_get(m_clients[client].model_handle, &current_ttl);
        if (current_ttl == 0xFF)
        {
            current_ttl = access_default_ttl_get();
        }

        offline_log_store_tx(seq_15bit, current_ttl, _statusSending, t_tx_us);

        log_uart_mesh_event("TX", node_address.address_start, m_tx_dst,
                            (uint32_t)seq_15bit, current_ttl, 0, 0, p_class, EXP_PAYLOAD_LEN);

        if (!_statusSending && m_tx_packet_count >= m_target_msgs)
        {
            periodic_message_allowed = false;
            m_run_done = true;
        }
    }
    else
    {
        m_tx_fail_total++;
        m_tx_fail_consec++;
        m_last_err = status;
        if (m_tx_fail_consec >= EXP_MAX_TX_FAIL)
        {
            periodic_message_allowed = false;
            m_run_aborted = true;
        }
    }
}

static void mesh_main_button_event_handler(uint32_t button_number)
{
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        case 1:
            m_status_requested = true;
            break;
        case 2:
            m_req_toggle = true;
            break;
        case 3:
            m_dump_requested = true;
            break;
        case 4:
            clearAllDisplayNodes = true;
            if (!periodic_message_allowed)
            {
                m_offline_count   = 0;
                m_offline_dropped = 0;
                m_drop_cache_cnt  = 0;
                m_relay_ok_cnt    = 0;
                m_rx_unique_cnt   = 0;
                m_mid_run_reset   = 0;
                exp_noinit_save(m_tn3_cfg_idx, 0, 0);
                SEGGER_RTT_WriteString(0, "# [ACTION] Nut 4: Da xoa nhanh bo dem log RAM, CACHE va co mid_reset.\r\n");
            }
            break;
        default:
            break;
    }
}


void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        case BSP_EVENT_KEY_1:
        case BSP_EVENT_KEY_2:
        case BSP_EVENT_KEY_3:
            mesh_main_button_event_handler(event - BSP_EVENT_KEY_0);
            break;
        default:
            break;
    }
}

static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;
    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void models_client_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(generic_level_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);
static void app_level_server_set_cb(const app_level_server_t * p_server, uint32_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, uint32_t * p_present_level);
static void app_level_server_transition_cb(const app_level_server_t * p_server, uint32_t transition_time_ms, uint32_t target_level, app_transition_type_t transition_type);

static nrf_mesh_evt_handler_t m_event_handler =
{
    .evt_cb = mesh_events_handle,
};

APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb,
                     app_level_server_transition_cb);

static void app_level_server_set_cb(const app_level_server_t * p_server, uint32_t present_level)
{
    uint16_t payload     = (uint16_t)present_level;
    uint8_t  traffic_bit = (payload >> 15) & 0x01;
    uint16_t seq_num     = payload & 0x7FFF;

    uint16_t raw_src = g_last_mesh_src_addr;
    uint16_t src_addr = raw_src;
    if (raw_src % 2 == 0 && raw_src > 1)
    {
        src_addr = raw_src - 1;
    }

    (void)seq_num;

    uint32_t display_stat = (traffic_bit == 1) ? _fireAlarmStatus : _normalStatus;
    insert_node((uint8_t)(src_addr & 0xFF), display_stat, false);

    refreshDisplayCounter = 1;
}

static void app_level_server_get_cb(const app_level_server_t * p_server, uint32_t * p_present_level)
{
    *p_present_level = (node_address.address_start << 16) + 6;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_level_server_Get_cb\n\r");
}

static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                           uint32_t transition_time_ms,
                                           uint32_t target_level,
                                           app_transition_type_t transition_type)
{
    (void)p_server; (void)transition_time_ms; (void)target_level; (void)transition_type;
}

static void app_model_init(void)
{
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Level Model handle: %d\n", m_level_server_0.server.model_handle);
}

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_ENABLED)
    {
        APP_ERROR_CHECK(app_level_value_restore(&m_level_server_0));
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    model_config_file_clear();
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, false);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
    clientProvisionChanged = true;
    clientProvisioned = true;
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif
    unicast_address_print();
    m_boot_preset_done = false;
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
    models_client_init_cb();
}

static void mesh_init(void)
{
    model_config_file_init();

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);

    if (status == NRF_SUCCESS)
    {
        status = model_config_file_config_apply();
    }

    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            model_config_file_clear();
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in persistent memory corrupted. Device starts unprovisioned.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_WARN, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Sensor Node (" FW_GIT_HASH ") -----\n");

    exp_noinit_check_on_boot();
    m_tn3_cfg_idx = EXP_STOCK_IDX;

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if EXP_ENABLE_TRIG
    nrf_gpio_cfg_output(TRIG_PIN);
    nrf_gpio_pin_clear(TRIG_PIN);
#endif

    bool erase_bonds;
    buttons_leds_init(&erase_bonds);
    ble_stack_init();

    exp_chain_mac_init();

    {
        uint32_t rr = 0;
        if (sd_power_reset_reason_get(&rr) == NRF_SUCCESS)
        {
            g_reset_reason = rr;
            (void)sd_power_reset_reason_clr(0xFFFFFFFFu);
        }
    }

    exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);

    if (m_mid_run_reset)
    {
        SEGGER_RTT_printf(0, "# [CANH_BAO_RESET] Phat hien reset giua chung khi dang o idx=%u (reset_reason=0x%08X)!\r\n",
                          (unsigned)m_tn3_cfg_idx, (unsigned)g_reset_reason);
    }

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    hal_led_blink_stop();
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
    mesh_init();

}

static void start(void)
{
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_DM_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));

        clientProvisionChanged = true;
        clientProvisioned = false;
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());
    nrf_mesh_evt_handler_add(&m_event_handler);
    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

static void trigger_egu30_event(void)
{
    NRF_EGU3->TASKS_TRIGGER[0] = 1;
}

static void all_parameters_initialization(void)
{
    m_present_level = _normalStatus;
    timerBCounter = 1;
    periodic_message_allowed = false;
    aht10Available = true;
    _clientTemperature = 0;
    _clientHumidity = 0;
    clearAllDisplayNodes = false;
}

void SWI3_EGU3_IRQHandler(void)
{
    NRF_EGU3->EVENTS_TRIGGERED[0] = 0;

    if (_getResetProvisionKey)
    {
        if (mesh_stack_is_device_provisioned())
        {
            mesh_config_clear();
            node_reset();
        }
    }

    if (m_present_level == _fireAlarmStatus)
    {
        publish_present_alarm_status(true);
    }
    else if (periodic_message_allowed)
    {
        publish_present_alarm_status(false);
    }
}

static void egu_init(void)
{
    NRF_EGU3->INTENSET = EGU_INTENSET_TRIGGERED0_Enabled << EGU_INTENSET_TRIGGERED0_Pos;
    NVIC_SetPriority(SWI3_EGU3_IRQn, 6);
    NVIC_EnableIRQ(SWI3_EGU3_IRQn);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    static char err_buf[160];
    const uint32_t ram_end = 0x20000000u + (NRF_FICR->INFO.RAM * 1024u);
    bool info_ok = (info >= 0x20000000u) && ((info & 3u) == 0u) && ((info + 12u) <= ram_end);

    if (id == NRF_FAULT_ID_SDK_ERROR && info_ok)
    {
        const error_info_t * e = (const error_info_t *)info;
        snprintf(err_buf, sizeof(err_buf),
                 "\r\n[FAULT] SDK_ERROR id=0x%08X pc=0x%08X err=0x%08X line=%u\r\n",
                 (unsigned int)id, (unsigned int)pc,
                 (unsigned int)e->err_code, (unsigned int)e->line_num);
    }
    else if (id == NRF_FAULT_ID_SDK_ASSERT && info_ok)
    {
        const assert_info_t * a = (const assert_info_t *)info;
        snprintf(err_buf, sizeof(err_buf),
                 "\r\n[FAULT] ASSERT id=0x%08X pc=0x%08X line=%u\r\n",
                 (unsigned int)id, (unsigned int)pc, (unsigned int)a->line_num);
    }
    else
    {
        snprintf(err_buf, sizeof(err_buf),
                 "\r\n[FAULT] id=0x%08X pc=0x%08X info=0x%08X\r\n",
                 (unsigned int)id, (unsigned int)pc, (unsigned int)info);
    }
    uart_puts(err_buf);
    SEGGER_RTT_WriteString(0, err_buf);

#if EXP_HALT_ON_FAULT
    while (1)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);
    }
#else
    NVIC_SystemReset();
    while (1) {}
#endif
}

nrfx_wdt_channel_id m_channel_id;

void wd_event_handle(void)
{
}

void wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrfx_wdt_config_t w_config = NRFX_WDT_DEAFULT_CONFIG;
    err_code = nrfx_wdt_init(&w_config, wd_event_handle);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
}

#define resetProvisionLED     18
#define resetProvisionBTN     13

bool checktoResetProvisioned(void)
{
    nrf_gpio_cfg_output(resetProvisionLED);
    nrf_gpio_cfg_input(resetProvisionBTN, NRF_GPIO_PIN_PULLUP);
    bool _checkresetProvision = false;
    uint64_t resetProvisionCounter = 0;
    while (!_checkresetProvision)
    {
        nrf_delay_ms(1);
        if (nrf_gpio_pin_read(resetProvisionBTN) == 0)
        {
            nrf_gpio_pin_clear(resetProvisionLED);
            resetProvisionCounter++;
            if (resetProvisionCounter > 5000)
            {
                _checkresetProvision = true;
            }
        }
        else
        {
            break;
        }
    }
    return _checkresetProvision;
}

void display_ClientStatus(void)
{
    if (clientProvisionChanged)
    {
        if (clientProvisioned)
        {
            char _buff[100];
            sprintf(_buff, "0x%04x", node_address.address_start);
            glcd_display_stationAddress(_buff);
        }
        else
        {
            clearSavedExtEEPROMNodes();
            clearNodesScreen();
            display_nodes();
            glcd_display_stationAddress("------");
        }
        clientProvisionChanged = false;
    }
}

static bool model_pub_dst_get(access_model_handle_t h, uint16_t * p_dst)
{
    dsm_handle_t dh = DSM_HANDLE_INVALID;
    nrf_mesh_address_t a;
    if (access_model_publish_address_get(h, &dh) == NRF_SUCCESS &&
        dh != DSM_HANDLE_INVALID &&
        dsm_address_get(dh, &a) == NRF_SUCCESS)
    {
        *p_dst = a.value;
        return true;
    }
    return false;
}

static void exp_cfg_capture(void)
{
    m_cfg_ttl_pub      = 0xFF;
    m_cfg_rtx          = 0xFF;
    m_cfg_txcnt        = 0xFF;
    m_cfg_txint_ms     = 0;
    m_cfg_dbm          = 127;
    m_cfg_ttl_eff      = EXP_DEFAULT_TTL_INIT;
    m_cfg_relay_tx     = 0xFF;
    m_cfg_relay_en     = 0xFF;
    m_cfg_relay_int_ms = 0;
    m_cfg_srv_pub      = -1;
    m_cfg_cli_pub      = -1;
    m_tx_dst           = 0xFFFF;
    m_cfg_match_ok     = false;

    if (!mesh_stack_is_device_provisioned())
    {
        return;
    }

    (void)access_model_publish_ttl_get(m_clients[0].model_handle, &m_cfg_ttl_pub);
    m_cfg_ttl_eff = (m_cfg_ttl_pub == 0xFF) ? access_default_ttl_get() : m_cfg_ttl_pub;

    access_publish_retransmit_t rtx;
    if (access_model_publish_retransmit_get(m_clients[0].model_handle, &rtx) == NRF_SUCCESS)
    {
        m_cfg_rtx = rtx.count;
    }
    else
    {
        m_cfg_rtx = 0xFF;
    }

    EXP_STEP("opt_read");
    mesh_opt_core_adv_t adv_r;
    radio_tx_power_t pw;
    if (mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &adv_r) == NRF_SUCCESS)
    {
        m_cfg_txcnt    = (uint8_t)adv_r.tx_count;
        m_cfg_txint_ms = (uint16_t)adv_r.tx_interval_ms;
    }
    if (mesh_opt_core_tx_power_get(CORE_TX_ROLE_ORIGINATOR, &pw) == NRF_SUCCESS)
    {
        m_cfg_dbm = (int8_t)pw;
    }

    mesh_opt_core_adv_t adv_rl;
    if (mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &adv_rl) == NRF_SUCCESS)
    {
        m_cfg_relay_tx     = (uint8_t)adv_rl.tx_count;
        m_cfg_relay_en     = adv_rl.enabled ? 1 : 0;
        m_cfg_relay_int_ms = (uint16_t)adv_rl.tx_interval_ms;
    }

    uint16_t dst = 0xFFFF;
    m_cfg_srv_pub = model_pub_dst_get(m_level_server_0.server.model_handle, &dst) ? 1 : 0;
    if (model_pub_dst_get(m_clients[0].model_handle, &dst))
    {
        m_cfg_cli_pub = 1;
        m_tx_dst = dst;
    }
    else
    {
        m_cfg_cli_pub = 0;
    }

    const exp_tn3_cfg_t * p = &EXP_TN3_TABLE[m_tn3_cfg_idx];
    bool tx_ok    = (m_cfg_txcnt == p->net_tx) && (m_cfg_txint_ms == 20);
#if EXP_RELAY_EXPECT_EN
    bool relay_ok = (m_cfg_relay_en == 1) && (m_cfg_relay_tx == (uint8_t)(p->relay_retx + 1)) && (m_cfg_relay_int_ms == 20);
#else
    bool relay_ok = (m_cfg_relay_en == 0);   /* TN5 vai tro LPN: relay phai TAT */
#endif
    bool ttl_ok   = (m_cfg_ttl_eff == p->ttl);
    m_cfg_match_ok = tx_ok && relay_ok && ttl_ok;
}

static void exp_cfg_line(char * out, size_t n)
{
    const exp_tn3_cfg_t * p_preset = &EXP_TN3_TABLE[m_tn3_cfg_idx];
    snprintf(out, n,
             "# CFG exp=%u;fw=%s;role=%s;num_msgs=%u;period_ms=%u;payload_len=%u;hops=%u;"
             "tn3_idx=%u;tn3_tag=r%u_t%u_ttl%u;cfg_match=%u;mid_reset=%u;run=%u;"
             "wifi_ch=%u;wifi_set_mbps=%u;run_k=%u;"
             "ttl_cfg=%u;ttl_pub=%u;ttl_eff=%u;pub_rtx=%u;"
             "net_tx_cfg=%u;net_tx_now=%u;net_int_ms=%u;"
             "tx_dbm_cfg=%d;tx_dbm_now=%d;"
             "relay_en=%u;relay_tx_cfg=%u;relay_tx_now=%u;relay_int_ms=%u;"
             "chain_idx=%d;mac=%02X%02X%02X%02X%02X%02X;"
             "opts=%s;srv_pub=%d;cli_pub=%d;cli_dst=0x%04X;tx_fail=%u;last_err=0x%08X;reset=0x%08X",
             (unsigned)EXP_ID, FW_GIT_HASH, m_role_tx ? "TX" : "RX",
             (unsigned)m_target_msgs, (unsigned)EXP_PERIOD_MS, (unsigned)EXP_PAYLOAD_LEN, (unsigned)g_exp_hops,
             (unsigned)m_tn3_cfg_idx,
             (unsigned)p_preset->relay_retx, (unsigned)p_preset->net_tx, (unsigned)p_preset->ttl,
             (unsigned)(m_cfg_match_ok ? 1u : 0u), (unsigned)m_mid_run_reset,
             (unsigned)(periodic_message_allowed ? 1u : 0u),
             (unsigned)m_wifi_ch, (unsigned)m_wifi_mbps, (unsigned)m_run_k,
             (unsigned)p_preset->ttl, (unsigned)m_cfg_ttl_pub, (unsigned)m_cfg_ttl_eff, (unsigned)m_cfg_rtx,
             (unsigned)p_preset->net_tx, (unsigned)m_cfg_txcnt, (unsigned)m_cfg_txint_ms,
             (int)EXP_TX_POWER_DBM, (int)m_cfg_dbm,
             (unsigned)m_cfg_relay_en, (unsigned)(p_preset->relay_retx + 1u), (unsigned)m_cfg_relay_tx, (unsigned)m_cfg_relay_int_ms,
             (int)m_my_chain_index,
             m_my_ble_addr.addr[5], m_my_ble_addr.addr[4], m_my_ble_addr.addr[3],
             m_my_ble_addr.addr[2], m_my_ble_addr.addr[1], m_my_ble_addr.addr[0],
             "read_verified",
             m_cfg_srv_pub, m_cfg_cli_pub, (unsigned)m_tx_dst,
             (unsigned)m_tx_fail_total, (unsigned)m_last_err, (unsigned)g_reset_reason);
}

static bool exp_tn3_apply_preset(uint8_t idx, bool blink_led)
{
    if (idx >= EXP_TN3_NUM_CFGS)
    {
        idx = 0;
    }
    m_tn3_cfg_idx = idx;
    exp_noinit_save(m_tn3_cfg_idx, periodic_message_allowed ? 1 : 0, m_mid_run_reset);

    const exp_tn3_cfg_t * p = &EXP_TN3_TABLE[idx];

    uint32_t err_ttl_def = access_default_ttl_set(p->ttl);
    uint32_t err_ttl_pub = access_model_publish_ttl_set(m_clients[0].model_handle, p->ttl);

    uint32_t err_org = NRF_ERROR_INTERNAL;
    mesh_opt_core_adv_t adv_org;
    if (mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &adv_org) == NRF_SUCCESS)
    {
        adv_org.enabled        = true;
        adv_org.tx_count       = (p->net_tx > 0) ? p->net_tx : 1;
        adv_org.tx_interval_ms = 20;
        err_org = mesh_opt_core_adv_set(CORE_TX_ROLE_ORIGINATOR, &adv_org);
    }

    uint32_t err_rel = NRF_ERROR_INTERNAL;
    mesh_opt_core_adv_t adv_rel;
    if (mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &adv_rel) == NRF_SUCCESS)
    {
        adv_rel.enabled        = (EXP_RELAY_EXPECT_EN != 0);
        adv_rel.tx_count       = (uint8_t)(p->relay_retx + 1u);
        adv_rel.tx_interval_ms = 20;
        err_rel = mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &adv_rel);
    }

    if (err_ttl_def != NRF_SUCCESS || err_ttl_pub != NRF_SUCCESS ||
        err_org != NRF_SUCCESS || err_rel != NRF_SUCCESS)
    {
        SEGGER_RTT_printf(0, "# [LOI_SET_CFG] idx=%u err: ttl_def=0x%X ttl_pub=0x%X org=0x%X rel=0x%X\r\n",
                          (unsigned)idx, (unsigned)err_ttl_def, (unsigned)err_ttl_pub,
                          (unsigned)err_org, (unsigned)err_rel);
    }

    exp_cfg_capture();
    exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
    SEGGER_RTT_WriteString(0, m_cfg_buf);
    SEGGER_RTT_WriteString(0, "\r\n");

    const int exp_ttl_rx = (int)p->ttl - ((int)g_exp_hops - 1);
    SEGGER_RTT_printf(0, "# [TN3_PRESET] idx=%u/23 | tag=r%u_t%u_ttl%u | Verify: ttl_eff=%u(can %u), net_tx_now=%u(can %u), relay_tx_now=%u(can %u) | match=%s\r\n",
                      (unsigned)idx,
                      (unsigned)p->relay_retx, (unsigned)p->net_tx, (unsigned)p->ttl,
                      (unsigned)m_cfg_ttl_eff, (unsigned)p->ttl,
                      (unsigned)m_cfg_txcnt, (unsigned)p->net_tx,
                      (unsigned)m_cfg_relay_tx, (unsigned)(p->relay_retx + 1u),
                      m_cfg_match_ok ? "OK" : "MISMATCH");

    if (exp_ttl_rx <= 0)
    {
        SEGGER_RTT_printf(0, "# [LUU Y KHOI 2] TTL=%u tren duong %u chang -> ttl_rx_exp=%d <= 0: PDR du kien tai GW = 0%% (PASS_TTL_BOUND).\r\n",
                          (unsigned)p->ttl, (unsigned)g_exp_hops, exp_ttl_rx);
    }

    if (blink_led)
    {
        hal_led_blink_stop();
        hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
        if (m_cfg_match_ok)
        {
            hal_led_blink_ms(BLUE_LED_pos, 150, (idx % 4) + 1);
        }
        else
        {
            hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
        }
    }
    return m_cfg_match_ok;
}

static void exp_tn3_select_preset(uint8_t target_idx)
{
    if (target_idx != EXP_STOCK_IDX)
    {
        SEGGER_RTT_printf(0, "# [LOI] TN4 khoa cau hinh stock idx=%u (r2_t2_ttl4). Yeu cau idx=%u bi tu choi.\r\n",
                          (unsigned)EXP_STOCK_IDX, (unsigned)target_idx);
        return;
    }
    if (periodic_message_allowed || (m_offline_count > 0))
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Dang chay hoac con log chua xa -> bam Nut 3 (xa log) hoac Nut 4 (xoa log) truoc khi doi idx.\r\n");
        return;
    }
    (void)exp_tn3_apply_preset(target_idx, true);
}


static void exp_toggle_run(void)
{
    if (periodic_message_allowed)
    {
        periodic_message_allowed = false;
        exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);
        SEGGER_RTT_WriteString(0, "# [INFO] Da dung phat.\r\n");
        return;
    }
    if (!mesh_stack_is_device_provisioned())
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Chua provision.\r\n");
        return;
    }


    if (m_mid_run_reset)
    {
        SEGGER_RTT_printf(0, "# [LOI_RESET] Board vua bi reset giua chung o idx=%u! Bam Nut 4 (clear) de xac nhan truoc khi chay lai.\r\n",
                          (unsigned)m_tn3_cfg_idx);
        return;
    }

    if (m_offline_count > 0)
    {
        SEGGER_RTT_printf(0, "# [LOI] Con %u dong chua xa! Bam Nut 3 de xa log (hoac Nut 4 de xoa) truoc khi bat run moi.\r\n",
                          (unsigned)m_offline_count);
        return;
    }

    EXP_STEP("cfg_capture");
    exp_cfg_capture();
    if (!m_cfg_match_ok)
    {
        (void)exp_tn3_apply_preset(m_tn3_cfg_idx, false);
    }

    const exp_tn3_cfg_t * p = &EXP_TN3_TABLE[m_tn3_cfg_idx];
    bool is_tx = (EXP_READ_CFG == 0) || (m_cfg_cli_pub == 1);

    if (!m_cfg_match_ok)
    {
        SEGGER_RTT_printf(0, "# [LOI_CHOT_CHAN] Stack LECH voi bang TN3 idx=%u (r%u_t%u_ttl%u): "
                             "ttl_eff=%u(can %u), net_tx_now=%u/%ums(can %u/20ms), relay_en=%u, relay_tx_now=%u/%ums(can %u/20ms) -> TU CHOI CHAY!\r\n",
                          (unsigned)m_tn3_cfg_idx,
                          (unsigned)p->relay_retx, (unsigned)p->net_tx, (unsigned)p->ttl,
                          (unsigned)m_cfg_ttl_eff, (unsigned)p->ttl,
                          (unsigned)m_cfg_txcnt, (unsigned)m_cfg_txint_ms, (unsigned)p->net_tx,
                          (unsigned)m_cfg_relay_en,
                          (unsigned)m_cfg_relay_tx, (unsigned)m_cfg_relay_int_ms, (unsigned)(p->relay_retx + 1u));
        hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
        return;
    }
    if (is_tx && (m_cfg_rtx != 0))
    {
        SEGGER_RTT_printf(0, "# [LOI_CHOT_CHAN] Publish Retransmit cua Client = %u (can 0). Dat trong nRF Mesh app roi bam Nut 2 lai.\r\n",
                          (unsigned)m_cfg_rtx);
        hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
        return;
    }

    if ((m_cfg_dbm != 127) && (m_cfg_dbm != (int8_t)EXP_TX_POWER_DBM))
    {
        SEGGER_RTT_printf(0, "# [CANH BAO] Cong suat phat doc duoc = %d dBm, can %d dBm.\r\n",
                          (int)m_cfg_dbm, (int)EXP_TX_POWER_DBM);
    }

    m_offline_dropped = 0;
    m_drop_cache_cnt  = 0;
    m_relay_ok_cnt    = 0;
    m_rx_unique_cnt   = 0;
    m_trig_cnt[1]     = 0;
    m_trig_cnt[2]     = 0;
    m_trig_cnt[3]     = 0;
    m_logging_enabled = true;
    exp_time_reset();

    exp_noinit_save(m_tn3_cfg_idx, 1, 0);

    if (!is_tx)
    {
        m_role_tx = false;
        exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
        SEGGER_RTT_WriteString(0, m_cfg_buf);
        SEGGER_RTT_printf(0, "\r\n# [ACTION] BO THU / RELAY (idx=%u: r%u_t%u_ttl%u): da xac nhan cau hinh & san sang nhan.\r\n",
                          (unsigned)m_tn3_cfg_idx,
                          (unsigned)p->relay_retx, (unsigned)p->net_tx, (unsigned)p->ttl);
        return;
    }

    m_tx_packet_count = 0;
    m_global_seq      = 0;
    m_tx_fail_total   = 0;
    m_tx_fail_consec  = 0;
    m_last_err        = 0;
    m_run_done        = false;
    m_run_aborted     = false;
    m_role_tx         = true;

    exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
    SEGGER_RTT_WriteString(0, m_cfg_buf);
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_printf(0, "# [ACTION] Bat dau run TN3 idx=%u (tag=r%u_t%u_ttl%u | muc tieu %u goi | ttl_eff=%u | net_tx_now=%u | relay_tx_now=%u)\r\n",
                      (unsigned)m_tn3_cfg_idx,
                      (unsigned)p->relay_retx, (unsigned)p->net_tx, (unsigned)p->ttl,
                      (unsigned)m_target_msgs, (unsigned)m_cfg_ttl_eff, (unsigned)m_cfg_txcnt, (unsigned)m_cfg_relay_tx);
    EXP_STEP("run_start");

    periodic_message_allowed = true;
}

static void exp_dump_log(void)
{
    bool aborted = false;
    uint16_t n = m_offline_count;

    m_logging_enabled = false;
    periodic_message_allowed = false;

    if (m_wifi_ch == 0xFF)
    {
        SEGGER_RTT_WriteString(0, "# [CANH_BAO] Chua khai bao nhan Wi-Fi (wf=CH,LOAD) -> file log se thieu thong tin nhieu! Van xa log (khong chan).\r\n");
    }
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    exp_cfg_capture();
    exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
    if (!rtt_write_line_reliable(m_cfg_buf, (unsigned)strlen(m_cfg_buf)) ||
        !rtt_write_line_reliable("\r\n", 2))
    {
        aborted = true;
    }

    const int ttl_origin = (m_cfg_ttl_eff != 0xFF) ? (int)m_cfg_ttl_eff : (int)EXP_DEFAULT_TTL_INIT;
    const int exp_ttl_rx = ttl_origin - ((int)g_exp_hops - 1);

    if (!aborted)
    {
        int l = snprintf(m_cfg_buf, sizeof(m_cfg_buf),
                         "# TRIG enabled=%u;tx_pulse=%u;fwd_pulse=%u;rx_pulse=%u;tx_ok=%u;tx_fail=%u;"
                         "width_us=%u;gap_us=%u;hops=%u;ttl_rx_exp=%d\r\n",
                         (unsigned)EXP_ENABLE_TRIG,
                         (unsigned)m_trig_cnt[EXP_TRIG_TX], (unsigned)m_trig_cnt[EXP_TRIG_FWD],
                         (unsigned)m_trig_cnt[EXP_TRIG_RX],
                         (unsigned)m_tx_packet_count, (unsigned)m_tx_fail_total,
                         (unsigned)TRIG_WIDTH_US, (unsigned)TRIG_GAP_US,
                         (unsigned)g_exp_hops, exp_ttl_rx);
        if (l > 0 && !rtt_write_line_reliable(m_cfg_buf, (unsigned)l))
        {
            aborted = true;
        }
    }

    if (!aborted)
    {
        uint32_t unique_seen = (m_relay_ok_cnt > 0) ? m_relay_ok_cnt : m_rx_unique_cnt;
        uint32_t total_seen  = unique_seen + m_drop_cache_cnt;
        uint32_t rho_permil  = (total_seen > 0) ? ((m_drop_cache_cnt * 1000u) / total_seen) : 0u;
        int l = snprintf(m_cfg_buf, sizeof(m_cfg_buf),
                         "# CACHE relay_ok=%u;rx_unique=%u;drop_cache=%u;total_seen=%u;rho=%u.%03u;rho_pct=%u.%u\r\n",
                         (unsigned)m_relay_ok_cnt, (unsigned)m_rx_unique_cnt,
                         (unsigned)m_drop_cache_cnt, (unsigned)total_seen,
                         (unsigned)(rho_permil / 1000u), (unsigned)(rho_permil % 1000u),
                         (unsigned)(rho_permil / 10u), (unsigned)(rho_permil % 10u));
        if (l > 0 && !rtt_write_line_reliable(m_cfg_buf, (unsigned)l))
        {
            aborted = true;
        }
    }


    int len = snprintf(m_line_buf, sizeof(m_line_buf),
                       "--- BAT DAU XA LOG CSV --- SO DONG: %u; BI TRAN BUFFER: %u ---\r\n",
                       (unsigned)n, (unsigned)m_offline_dropped);
    if (!aborted && len > 0 && !rtt_write_line_reliable(m_line_buf, (unsigned)len))
    {
        aborted = true;
    }

    static const char header[] =
        "t_local_us,node_id,event,src,dst,seq,ttl_tx,ttl_rx,rssi,class,payload_len,DROP_CACHE\r\n";
    if (!aborted && !rtt_write_line_reliable(header, sizeof(header) - 1))
    {
        aborted = true;
    }

    uint16_t my_id = (uint16_t)(node_address.address_start & 0xFFFF);

    for (uint16_t i = 0; (i < n) && !aborted; i++)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);

        uint16_t seq      = m_offline_logs[i].seq_alarm & 0x7FFF;
        bool     is_alarm = (m_offline_logs[i].seq_alarm >> 15) & 0x01;
        uint8_t  evt      = UNPACK_EVT(m_offline_logs[i].ttl_evt);
        uint8_t  ttl_val  = UNPACK_TTL(m_offline_logs[i].ttl_evt);
        uint8_t  dc_val   = m_offline_logs[i].drop_cache;

        const char * evt_str = (evt == EXP_EVT_TX) ? "TX" :
                               ((evt == EXP_EVT_RELAY) ? "RELAY" : "RX");

        uint16_t dst      = (evt == EXP_EVT_RX) ? my_id : m_tx_dst;
        uint8_t  ttl_tx   = (evt == EXP_EVT_TX) ? ttl_val :
                            ((evt == EXP_EVT_RELAY && ttl_val > 0) ? (uint8_t)(ttl_val - 1) : 0);
        uint8_t  ttl_rx   = (evt == EXP_EVT_TX) ? 0 : ttl_val;

        len = snprintf(m_line_buf, sizeof(m_line_buf),
                       "%u,0x%04X,%s,0x%04X,0x%04X,%u,%u,%u,%d,%s,%u,%u\r\n",
                       (unsigned)m_offline_logs[i].t_us,
                       (unsigned)my_id,
                       evt_str,
                       (unsigned)m_offline_logs[i].src,
                       (unsigned)dst,
                       (unsigned)seq,
                       (unsigned)ttl_tx,
                       (unsigned)ttl_rx,
                       (int)m_offline_logs[i].rssi,
                       is_alarm ? "ALARM" : "TELEMETRY",
                       (unsigned)EXP_PAYLOAD_LEN,
                       (unsigned)dc_val);

        if (len > 0 && !rtt_write_line_reliable(m_line_buf, (unsigned)len))
        {
            aborted = true;
        }
    }

    if (aborted)
    {
        SEGGER_RTT_WriteString(0, "\r\n[LOI] RTT chua mo hoac bi nghen! Du lieu van giu trong RAM, hay bam Nut 3 lai.\r\n");
        return;
    }

    (void)rtt_write_line_reliable("--- KET THUC XA LOG ---\r\n", sizeof("--- KET THUC XA LOG ---\r\n") - 1);

    m_offline_count   = 0;
    m_offline_dropped = 0;
    m_logging_enabled = true;
    m_wifi_ch   = 0xFF;
    m_wifi_mbps = 0;
    m_run_k     = 0;
    exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);
}

/* ---------- TN5 ---------- */
__attribute__((weak)) void exp_tn5_lcd_sleep(void)
{
    /* TODO (phu thuoc driver st7789_drivers): gui SLPIN (0x10) + DISPOFF (0x28) va tat chan den nen.
     * Ban co the dinh nghia lai ham nay o noi khac (ham weak se bi ghi de). */
}

#if EXP_TN5_AUTORUN && !EXP_TN5_DS
/* Khong co nut/RTT luc do (J-Link phai thao): tu goi exp_toggle_run() cho den khi cau hinh khop */
static void exp_tn5_autorun_service(void)
{
    static uint32_t next_try_s = EXP_TN5_AUTORUN_DELAY_S;
    static bool     done       = false;

    if (done) { return; }
    if (periodic_message_allowed) { done = true; return; }
    if (!m_boot_preset_done || !mesh_stack_is_device_provisioned()) { return; }
    if (m_secondCounter < next_try_s) { return; }

    next_try_s = m_secondCounter + 10;
    exp_toggle_run();     /* vai tro TX: bat periodic_message_allowed; vai tro RX/relay: xac nhan cau hinh */
    if (periodic_message_allowed || m_cfg_match_ok) { done = true; }
}
#endif

/* Goi moi 1 s tu vong lap chinh: duty-cycle soi dot MQ-2 + doc cam bien. Khong dung LCD/RTT. */
static void exp_tn5_second_tick(void)
{
    static uint32_t next_sense_s = 0;
    const uint32_t  s = m_secondCounter;

#if EXP_TN5_HEATER_CTRL
    static bool     heater_on = false;
    static uint32_t mq2_win   = 0xFFFFFFFFu;
    const uint32_t  ph        = s % EXP_TN5_MQ2_PERIOD_S;
    const bool      want_on   = (ph < EXP_TN5_MQ2_ON_S);

    if (want_on != heater_on)   /* theo trang thai, khong theo dung tick: mat 1 tick van khong ket */
    {
        heater_on = want_on;
        mq2_heater_set(heater_on);
        exp_trig_marker(heater_on ? EXP_TRIG_HEAT_ON : EXP_TRIG_HEAT_OFF);
    }
#endif

    if (s >= next_sense_s)
    {
        next_sense_s = s + EXP_TN5_SENSE_PERIOD_S;
        exp_trig_marker(EXP_TRIG_SENSE);

        if (aht10Available)
        {
            _clientTemperature = aht10_readTemperature(AHT10_FORCE_READ_DATA);
            _clientHumidity    = aht10_readHumidity(false);
        }
        else
        {
            aht10Available = aht10_begin();
        }

#if EXP_TN5_HEATER_CTRL
        const uint32_t win = s / EXP_TN5_MQ2_PERIOD_S;
        if (mq2Available && heater_on && (ph >= EXP_TN5_MQ2_SETTLE_S) && (win != mq2_win))
        {
            mq2_win = win;   /* 1 mau hop le moi cua so soi dot */
            _clientMQ2Value = read_mq2_sensor();
        }
#else
        if (mq2Available)
        {
            _clientMQ2Value = read_mq2_sensor();   /* soi dot luon nong */
        }
#endif
    }
}

#if EXP_TN5_DS
/* Bien the A: ngu sau. Khong tro lai. */
static void exp_tn5_enter_deep_sleep(void)
{
    mq2_heater_set(false);
    hal_led_blink_stop();
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    uint32_t e = nrf_mesh_disable();
    SEGGER_RTT_printf(0, "# [TN5] nrf_mesh_disable = 0x%X\r\n", (unsigned)e);
    (void)sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    SEGGER_RTT_WriteString(0, "# [TN5] NGU SAU. Thao J-Link, cap nguon qua PPK2, bo 30 s dau.\r\n");
    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
#endif

static void exp_requests_service(void)
{
    if (!m_boot_preset_done && (m_secondCounter >= 2) && mesh_stack_is_device_provisioned())
    {
        m_boot_preset_done = true;
        (void)exp_tn3_apply_preset(m_tn3_cfg_idx, false);
    }

    if (m_req_tn3_target >= 0)
    {
        uint8_t target = (uint8_t)m_req_tn3_target;
        m_req_tn3_target = -1;
        m_boot_preset_done = true;
        exp_tn3_select_preset(target);
    }

#if EXP_TN5_ENERGY && EXP_TN5_AUTORUN && !EXP_TN5_DS
    exp_tn5_autorun_service();
#endif

    if (m_status_requested)
    {
        m_status_requested = false;
        exp_cfg_capture();
        exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
        SEGGER_RTT_WriteString(0, m_cfg_buf);
        SEGGER_RTT_WriteString(0, "\r\n");
    }


    if (m_req_toggle)
    {
        m_req_toggle = false;
        exp_toggle_run();
    }

    if (m_run_done)
    {
        m_run_done = false;
        exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);
        SEGGER_RTT_printf(0, "# [INFO] Du %u goi TN3 (idx=%u). Cam J-Link + RTT Viewer roi bam Nut 3 de xa log.\r\n",
                          (unsigned)m_target_msgs, (unsigned)m_tn3_cfg_idx);
    }

    if (m_run_aborted)
    {
        m_run_aborted = false;
        exp_noinit_save(m_tn3_cfg_idx, 0, m_mid_run_reset);
        SEGGER_RTT_printf(0, "# [LOI] Gui loi %u lan lien tiep (err=0x%08X). Dung run. "
                             "0x5 = chua co publish address, 0x8 = chua provision.\r\n",
                          (unsigned)EXP_MAX_TX_FAIL, (unsigned)m_last_err);
    }

    if (m_dump_requested)
    {
        SEGGER_RTT_WriteString(0, "[ACTION] Nhan Nut 3 -> Dang xa log...\r\n");
        exp_dump_log();
        m_dump_requested = false;
    }
}

int main(void)
{
    mq2_heater_init();   /* viec dau tien: soi dot MQ-2 TAT */
    log_config_init();
    SEGGER_RTT_printf(0, "# TN5_CFG fw=%s;deepsleep=%d;role=%d;relay_retx=%d;cfg_idx=%d;trig=%d;sense_s=%d;"
                         "mq2_on_s=%d;mq2_period_s=%d;mq2_settle_s=%d;tx_dbm=%d;lcd_sleep=%d;heater_ctrl=%d\r\n",
                      FW_GIT_HASH, (int)EXP_TN5_DS, (int)EXP_TN5_ROLE, (int)EXP_TN5_RELAY_RETX,
                      (int)EXP_TN5_CFG_IDX, (int)EXP_ENABLE_TRIG, (int)EXP_TN5_SENSE_PERIOD_S,
                      (int)EXP_TN5_MQ2_ON_S, (int)EXP_TN5_MQ2_PERIOD_S, (int)EXP_TN5_MQ2_SETTLE_S,
                      (int)EXP_TX_POWER_DBM, (int)EXP_TN5_LCD_SLEEP, (int)EXP_TN5_HEATER_CTRL);
    SEGGER_RTT_WriteString(0, "-> Qua log_config_init\r\n");

    _getResetProvisionKey = checktoResetProvisioned();
    nrf_gpio_cfg_default(resetProvisionLED);   /* checktoResetProvisioned() de chan LED o muc output-thap */
    SEGGER_RTT_WriteString(0, "-> Qua checktoResetProvisioned\r\n");

    all_parameters_initialization();

#if !EXP_TN5_DS
    glcd_display_initialization();
    SEGGER_RTT_WriteString(0, "-> Qua glcd_init\r\n");

    glcd_printText("Nodes:", 1, 40, TFT_WHITE, TFT_BLACK);
    SEGGER_RTT_WriteString(0, "-> Qua glcd_printText\r\n");
    exp_tn5_lcd_sleep();
#endif

    initialize();
    SEGGER_RTT_WriteString(0, "-> Qua initialize\r\n");

    start();
    SEGGER_RTT_WriteString(0, "-> Qua start (Mesh da chay)\r\n");
    (void)sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
#if EXP_TN5_DS
    exp_tn5_enter_deep_sleep();   /* khong tro lai: bo qua WDT, timer 1 s, TWI, cam bien */
#endif

    wdt_init();
    SEGGER_RTT_WriteString(0, "-> Qua wdt_init\r\n");

    custom_timer_init();
    SEGGER_RTT_WriteString(0, "-> Qua custom_timer_init\r\n");

    egu_init();
    SEGGER_RTT_WriteString(0, "-> Qua egu_init\r\n");

    twi_init();
    SEGGER_RTT_WriteString(0, "-> Qua twi_init\r\n");

    static uint8_t _i2cDevices[255];
    detect_i2cDevice(_i2cDevices);
    SEGGER_RTT_WriteString(0, "-> Qua detect_i2cDevice\r\n");

    loadExtEEPROM_Nodes();
    SEGGER_RTT_WriteString(0, "-> Qua loadExtEEPROM_Nodes\r\n");

    nrf_drv_wdt_channel_feed(m_channel_id);

    if (aht10_begin() == true)
    {
        aht10Available = true;
    }
    SEGGER_RTT_WriteString(0, "-> Qua aht10_begin\r\n");

    if (mq2Available)
    {
        _clientMQ2Value = read_mq2_sensor();
    }

    if (aht10Available)
    {
        _clientTemperature = aht10_readTemperature(AHT10_FORCE_READ_DATA);
        _clientHumidity = aht10_readHumidity(false);
    }

    if (_getResetProvisionKey)
    {
        trigger_egu30_event();
    }

    SEGGER_RTT_WriteString(0, "-> Vao vong lap for(;;) thanh cong! (" FW_GIT_HASH ", se tu ap preset o s=2)\r\n");

    for (;;)
    {
        exp_requests_service();

        if (m_second_changed)
        {
            /* TN4: phat TX ngay dau moi tick (truoc LCD/I2C) de chu ky 1000 ms deu giua cac run. */
            if (periodic_message_allowed && mesh_stack_is_device_provisioned())
            {
                trigger_egu30_event();
            }

            exp_tn5_second_tick();   /* TN5: khong RTT/LCD/display_nodes moi giay */

            m_second_changed = false;
        }

        nrf_drv_wdt_channel_feed(m_channel_id);
        (void)sd_app_evt_wait();
    }
}