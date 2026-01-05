#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include <stdint.h>

#include <lvgl.h>

#include <zmk/display/status_screen.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>

#define DIM_AFTER_MS        180000u  /* 30秒無操作で暗くする（好きに変更OK） */
#define CONTRAST_BRIGHT     0xCF     /* 明るい（好みで） */
#define CONTRAST_DIM        0x05     /* 暗い（好みで） */

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/ble.h>
#endif

#include "cattail_header/cattail_1.h"
#include "cattail_header/cattail_2.h"
#include "cattail_header/cattail_3.h"
#include "cattail_header/cattail_4.h"
// #include "cattail_header/cattail_5.h"
// #include "cattail_header/cattail_6.h"

#include "cattail_header/cattail_dim.h"

/* USB状態で“充電中っぽい”を推定（後でVBUS GPIOに差し替え可能） */
#if defined(__has_include)
  #if __has_include(<zmk/usb.h>)
    #include <zmk/usb.h>
    #define HAVE_ZMK_USB_H 1
  #else
    #define HAVE_ZMK_USB_H 0
  #endif
#else
  #define HAVE_ZMK_USB_H 0
#endif

#define DEBUG_ENABLE 0

#if DEBUG_ENABLE == 1
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(roBa_screen, LOG_LEVEL_DBG);
#endif

/* ===============================
 * keymap display-name (自動追従)
 * =============================== */
#if DT_HAS_COMPAT_STATUS_OKAY(zmk_keymap)
#define KEYMAP_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zmk_keymap)
#define LAYER_DNAME(node_id) DT_PROP_OR(node_id, display_name, ""),

static const char *const layer_display_names[] = {
    DT_FOREACH_CHILD(KEYMAP_NODE, LAYER_DNAME)
};

#define LAYER_COUNT ((int)ARRAY_SIZE(layer_display_names))

static const char *get_layer_display_name(int layer) {
    if (layer >= 0 && layer < LAYER_COUNT) {
        return layer_display_names[layer] ? layer_display_names[layer] : "";
    }
    return "";
}
#else
#define LAYER_COUNT 0
static const char *get_layer_display_name(int layer) {
    ARG_UNUSED(layer);
    return "";
}
#endif

/* ===============================
 * OLED (SSD1306)
 * =============================== */
#define OLED_W 128
#define OLED_H 32
#define OLED_BUF_SIZE (OLED_W * OLED_H / 8)

static const struct device *oled;
static uint8_t oled_buf[OLED_BUF_SIZE];

static bool g_dimmed;

static void oled_init(void) {
    if (oled) return;

    oled = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(oled)) {
#if DEBUG_ENABLE == 1
        LOG_ERR("display not ready");
#endif
        oled = NULL;
        return;
    }

    display_blanking_off(oled);
#if DEBUG_ENABLE == 1
    LOG_DBG("display ready");
#endif
}

static void oled_push(const uint8_t *buf) {
    if (!oled) return;

    struct display_buffer_descriptor desc = {
        .width = OLED_W,
        .height = OLED_H,
        .pitch = OLED_W,
        .buf_size = OLED_BUF_SIZE,
    };

    (void)display_write(oled, 0, 0, &desc, buf);
}

static void apply_dim(bool dim) {
    if (!oled) return;
    if (g_dimmed == dim) return;

    int rc = display_set_contrast(oled, dim ? CONTRAST_DIM : CONTRAST_BRIGHT);
    if (rc) {
#if DEBUG_ENABLE == 1
        LOG_DBG("display_set_contrast rc=%d", rc);
#endif
    } else {
        g_dimmed = dim;
    }
}


/* ===============================
 * Canvas (portrait) : ALPHA_1BIT
 * =============================== */
#define CANVAS_H 128
#define CANVAS_W 32

static lv_obj_t *canvas;

#define CANVAS_BITMAP_BYTES (((CANVAS_W * CANVAS_H) + 7) / 8)
static uint8_t canvas_buf[CANVAS_BITMAP_BYTES];

/* 回転方向：true=90°CW, false=90°CCW(=270°CW) */
static const bool ROTATE_CW_90 =false;

/* イベントから直接描画しない 通知用 */
static atomic_bool g_hint_dirty = true;

/* ===============================
 * 省電力：idle/active 切替
 *   idle:   1000ms (1Hz)
 *   active:  200ms (5Hz)
 *   hold:   3000ms
 * =============================== */
#define PERIOD_IDLE_MS    1000u
#define PERIOD_ACTIVE_MS   200u
#define ACTIVE_HOLD_MS    3000u

static atomic_uint_fast32_t g_last_activity_ms;
static atomic_uint_fast32_t g_timer_period_ms;
static lv_timer_t *g_refresh_timer;

#ifndef lv_timer_ready
#define lv_timer_ready(t) lv_timer_reset(t)
#endif

static void set_refresh_period(uint32_t period_ms, bool ready_now) {
    if (!g_refresh_timer) return;

    uint32_t cur = (uint32_t)atomic_load(&g_timer_period_ms);
    if (cur != period_ms) {
        lv_timer_set_period(g_refresh_timer, period_ms);
        atomic_store(&g_timer_period_ms, period_ms);
    }
    if (ready_now) {
        lv_timer_ready(g_refresh_timer);
    }
}

static void mark_dirty_only(void) {
    atomic_store(&g_hint_dirty, true);
}

static void mark_activity(void) {
    uint32_t now = k_uptime_get_32();
    atomic_store(&g_last_activity_ms, now);
    atomic_store(&g_hint_dirty, true);

    apply_dim(false);                 /* ← 追加：操作が来たら明るさ復帰 */

    set_refresh_period(PERIOD_ACTIVE_MS, true);
}

/* ===============================
 * helper: 先頭N文字切り出し
 * =============================== */
static void take_prefix(const char *in, char *out, size_t out_sz, size_t n) {
    if (!out || out_sz == 0) return;
    out[0] = '\0';
    if (!in) return;

    size_t o = 0;
    for (size_t i = 0; in[i] != '\0' && o + 1 < out_sz; i++) {
        out[o++] = in[i];
        if (o >= n) break;
    }
    out[o] = '\0';
}

/* ===============================
 * VLSB helper (SSD1306 page format)
 * =============================== */
static inline void set_px_vlsb(uint8_t *buf, int x, int y, bool on) {
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    int idx = x + (y / 8) * OLED_W;
    uint8_t bit = 1u << (y & 7);
    if (on) buf[idx] |= bit;
    else    buf[idx] &= (uint8_t)~bit;
}

/* Canvas ALPHA_1BIT pixel get (MSB-first) */
static inline bool canvas_get_px_alpha1(const uint8_t *buf, int w, int x, int y) {
    int i = y * w + x;
    uint8_t b = buf[i >> 3];
    uint8_t m = (uint8_t)(1u << (7 - (i & 7)));
    return (b & m) != 0;
}

/* Canvas ALPHA_1BIT pixel set (MSB-first) */
static inline void canvas_set_px_alpha1(uint8_t *buf, int w, int x, int y, bool on) {
    if (x < 0 || x >= w || y < 0 || y >= CANVAS_H) return;
    int i = y * w + x;
    uint8_t *b = &buf[i >> 3];
    uint8_t m = (uint8_t)(1u << (7 - (i & 7)));
    if (on) *b |= m;
    else    *b &= (uint8_t)~m;
}

/* Canvas(32x128) → OLED(128x32) 90°回転 + VLSB */
static void canvas_to_oled_rot(uint8_t *dst) {
    memset(dst, 0, OLED_BUF_SIZE);

    for (int y = 0; y < CANVAS_H; y++) {
        for (int x = 0; x < CANVAS_W; x++) {
            bool on = canvas_get_px_alpha1(canvas_buf, CANVAS_W, x, y);

            int ox, oy;
            if (ROTATE_CW_90) {
                ox = y;
                oy = (CANVAS_W - 1) - x;
            } else {
                ox = (CANVAS_H - 1) - y;
                oy = x;
            }
            set_px_vlsb(dst, ox, oy, on);
        }
    }
}

/* ===============================
 * USB / charging
 * =============================== */
#define BATT_UNKNOWN (-1)
#define PERIPHERAL_SOURCE 0 

static atomic_int g_soc_left      = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* 0-100, -1=unknown */
static atomic_int g_soc_right     = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* 0-100, -1=unknown */
static atomic_int g_chg_left      = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* 0/1,  -1=unknown */
static atomic_int g_chg_right     = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* 0/1,  -1=unknown */
static atomic_int g_usb_left      = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* 0/1,  -1=unknown */
static atomic_int g_usb_right     = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* 0/1,  -1=unknown */
static atomic_int g_periph_source  = ATOMIC_VAR_INIT(BATT_UNKNOWN); /* peripheral src id, -1=unknown */

typedef struct {
    int layer;
    int ble;
    int l_batt;      /* 0-100 */
    int r_batt;      /* 0-100 */
    bool l_usb;
    bool r_usb;
    bool l_charging; /* “USB接続中 かつ batt<100” を充電中扱い */
    bool r_charging; /* “USB接続中 かつ batt<100” を充電中扱い */
} ui_state_t;

static bool usb_connected_guess(void) {
#if HAVE_ZMK_USB_H && IS_ENABLED(CONFIG_ZMK_USB)
    return zmk_usb_get_conn_state() != 0;
#else
    return false;
#endif
}

#if DT_NODE_EXISTS(DT_NODELABEL(chg_central))
static const struct gpio_dt_spec chg_central_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(r_chg), gpios);
#endif

static bool central_gpio_charge_init(void) {
#if DT_NODE_EXISTS(DT_NODELABEL(chg_central))
    if (device_is_ready(chg_central_gpio.port)) {
        int rc = gpio_pin_configure_dt(&chg_central_gpio, GPIO_INPUT);
        if (rc < 0) {
            atomic_store(&g_chg_right, BATT_UNKNOWN);
        }
    } else {
        atomic_store(&g_chg_right, BATT_UNKNOWN);
    }

    return true;
#endif
}

static void update_central_chg_from_gpio(void) {
#if DT_NODE_EXISTS(DT_NODELABEL(chg_central))
    if (!device_is_ready(chg_central_gpio.port)) {
        atomic_store(&g_chg_right, BATT_UNKNOWN);
        return;
    }

    int v = gpio_pin_get_dt(&chg_central_gpio); // 0/1 or negative
    if (v < 0) {
        atomic_store(&g_chg_right, BATT_UNKNOWN);
        return;
    }

    bool charging = (v != 0);
    bool usb = usb_connected_guess();

    atomic_store(&g_usb_right, usb);
    atomic_store(&g_chg_right, charging ? 1 : 0);
#endif
}

/* ===============================
 * Data snapshot
 * =============================== */
static int clamp_soc(int soc) {
    if (soc < 0) return -1;
    if (soc > 100) return 100;
    return soc;
}

static int top_layer_index(void) {
  zmk_keymap_layers_state_t st = zmk_keymap_layer_state();
  for (int i = 31; i >= 0; i--) {
    if (st & (1u << i)) return i;
  }
  return 0;
}

/* read_state() はキャッシュから読むだけに寄せる */
static ui_state_t read_state(void) {
    ui_state_t s = {0};

    s.layer = top_layer_index();
#if IS_ENABLED(CONFIG_ZMK_BLE)
    s.ble = zmk_ble_active_profile_index() + 1;
#endif

    /* Left battery: eventキャッシュ優先、無ければAPI値 */
    int l_soc = atomic_load(&g_soc_left);
    if (l_soc < 0) {
        l_soc = (int)zmk_battery_state_of_charge();
    }
    s.l_batt = clamp_soc(l_soc);
    int r_soc= atomic_load(&g_soc_right);
    s.r_batt = clamp_soc(r_soc);              /* ← batt_listenerが更新 */

    int l_chg    =atomic_load(&g_chg_left);
    s.l_charging =(l_chg == 1);
    int l_usb    =atomic_load(&g_usb_left);
    s.l_usb      =(l_usb == 1);

    int r_chg    =atomic_load(&g_chg_right); /* ← input callbackが更新 */
    s.r_charging =(r_chg == 1);
    int r_usb    =atomic_load(&g_usb_right);
    s.r_usb      =(r_usb == 1);

    return s;
}


static bool state_equal(ui_state_t a, ui_state_t b) {
    return a.layer      == b.layer && 
           a.ble        == b.ble && 
           a.l_batt     == b.l_batt &&
           a.r_batt     == b.r_batt &&
           a.l_charging == b.l_charging &&
           a.r_charging == b.r_charging;
}

/* ===============================
 * Tiny 3x5 digits (for battery percent)
 * =============================== */
static const uint8_t dig3x5[10][5] = {
    {0b111,0b101,0b101,0b101,0b111}, /*0*/
    {0b010,0b110,0b010,0b010,0b111}, /*1*/
    {0b111,0b001,0b111,0b100,0b111}, /*2*/
    {0b111,0b001,0b111,0b001,0b111}, /*3*/
    {0b101,0b101,0b111,0b001,0b001}, /*4*/
    {0b111,0b100,0b111,0b001,0b111}, /*5*/
    {0b111,0b100,0b111,0b101,0b111}, /*6*/
    {0b111,0b001,0b001,0b001,0b001}, /*7*/
    {0b111,0b101,0b111,0b101,0b111}, /*8*/
    {0b111,0b101,0b111,0b001,0b111}, /*9*/
};

static void draw_digit_3x5(int x, int y, int d) {
    if (d < 0 || d > 9) return;
    for (int ry = 0; ry < 5; ry++) {
        uint8_t row = dig3x5[d][ry];
        for (int rx = 0; rx < 3; rx++) {
            bool on = (row & (1u << (2 - rx))) != 0;
            if (on) canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + rx, y + ry, true);
        }
    }
}

static void draw_percent_mark(int x, int y) {
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 0, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 0, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 0, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 1, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 2, y + 1, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 4, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 2, y + 4, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 4, true);
}

static void draw_batt_percent_small(int x, int y, int pct) {
    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;

    if (pct == 100) {
        draw_digit_3x5(x + 0, y, 1);
        draw_digit_3x5(x + 4, y, 0);
        draw_digit_3x5(x + 8, y, 0);
        // draw_percent_mark(x + 12, y);
    } else {
        int tens = pct / 10;
        int ones = pct % 10;
        if (tens > 0) {
            draw_digit_3x5(x + 0, y, tens);
            draw_digit_3x5(x + 4, y, ones);
            draw_percent_mark(x + 8, y);
        } else {
            draw_digit_3x5(x + 4, y, ones);
            draw_percent_mark(x + 8, y);
        }
    }
}

/* ===============================
 * Battery icon + 4-step gauge
 * =============================== */
static int batt_level4(int pct) {
    if (pct >= 90) return 8;
    if (pct >= 70) return 6;
    if (pct >= 50) return 4;
    if (pct >= 25) return 2;
    return 0;
}

static void draw_charge_bolt_small(int x, int y) {
    /* 4x5 くらいの小さい⚡ */
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 2, y + 0, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 1, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 2, y + 1, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 2, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 4, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 5, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 4, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 4, true);
}


static void draw_usb_connection_small(int x, int y) {
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 6, y + 1, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 5, y + 2, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 2, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 4, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 5, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 7, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 8, y + 3, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + 4, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 1, y + 4, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 3, y + 4, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 4, y + 5, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 5, y + 6, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 6, y + 6, true);
    canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 7, y + 6, true);
}

static void draw_batt_icon_and_gauge(int x, int y, int pct, bool charging) {
    ARG_UNUSED(charging);

    const int body_w = 10, body_h = 7;
    const int nub_w = 2, nub_h = 3;

    for (int dx = 0; dx < body_w; dx++) {
        canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + dx, y + 0, true);
        canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + dx, y + body_h - 1, true);
    }
    //for (int dy = 0; dy < body_h; dy++) {
    for (int dy = 1; dy < body_h-1; dy++) {
        canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + 0, y + dy, true);
        canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + body_w - 1, y + dy, true);
    }
    for (int dx = 0; dx < nub_w; dx++) {
        for (int dy = 0; dy < nub_h; dy++) {
            canvas_set_px_alpha1(canvas_buf, CANVAS_W, x + body_w + dx, y + 2 + dy, true);
        }
    }

    int lv4 = batt_level4(pct);

    int bx0 = x + 1;
    int by0 = y + 1;
#define BOTTLE_H 5

    for (int dx = 0; dx < lv4; dx++) {
      for (int dy = 0; dy < BOTTLE_H; dy++) {
          canvas_set_px_alpha1(canvas_buf, CANVAS_W, bx0 +dx, by0 +dy, true);
        }
    }
}

typedef struct {
    const uint8_t *data; /* 32x64 1bpp MSB-first, row-major */
} art_frame_t;

static const art_frame_t g_frames[] = {
    { cattail_1 },
    { cattail_2 },
    { cattail_3 },
    { cattail_4 },
//    { cattail_5 },
//    { cattail_6 },
};

#define FRAME_COUNT     (ARRAY_SIZE(g_frames))
#define ANIM_PERIOD_MS 1000u   /* 1Hz固定 */
static uint8_t g_frame_idx;
static lv_timer_t *g_anim_timer;

#define ART_H 64
#define ART_W 32
#define ART_ROW_BYTES (ART_W / 8) /* 4 */

static void blit_art_32x64_to_canvas(int dst_x, int dst_y, const uint8_t *src) {
    /* x=0固定が一番安全（32幅なので行が常に4byte境界） */
    if (dst_x != 0) return;
    if (dst_y < 0 || (dst_y + ART_H) > CANVAS_H) return;

    for (int r = 0; r < ART_H; r++) {
        int bit_index = (dst_y + r) * CANVAS_W + dst_x; /* dst_x=0 */
        uint8_t *dst = &canvas_buf[bit_index >> 3];
        memcpy(dst, &src[r * ART_ROW_BYTES], ART_ROW_BYTES);
    }
}

static void on_dim_changed(bool dim) {
    (void)dim;
    g_frame_idx = 0;                      // ← 復帰時も最初からにしたいなら
    atomic_store(&g_hint_dirty, true);    // ← 次のrefreshで必ず描く

    /* refreshが1Hzでも即反映したい */
    if (g_refresh_timer) {
        lv_timer_reset(g_refresh_timer);  // lv_timer_readyが無い環境でもOK
    }
}

/* ===============================
 * Drawing (6段)
 * =============================== */
static void draw_separator(int y) {
    for (int x = 0; x < CANVAS_W; x++) {
        canvas_set_px_alpha1(canvas_buf, CANVAS_W, x, y, true);
    }
}

static void build_canvas_from_state(ui_state_t st) {
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);

    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);

    dsc.opa   = LV_OPA_COVER;
    dsc.color = lv_color_white();
    dsc.align = LV_TEXT_ALIGN_LEFT;
    dsc.font  = &lv_font_unscii_8;
    dsc.flag  = LV_TEXT_FLAG_EXPAND;

    /* 1段目：バッテリー(L / R) */
    // int w =batt_percent_width_px(st.batt);
    char line1[16] ="L  R";
    lv_canvas_draw_text(canvas, 0, 0, CANVAS_W, &dsc, line1);

    draw_batt_icon_and_gauge(2, 10, st.l_batt, false);
#if 1
    if(st.l_charging) {
      draw_charge_bolt_small(8, 1);
    } else if(st.l_usb) {
      draw_usb_connection_small(8, 1);
    }
#endif
    draw_batt_percent_small(3, 18, st.l_batt);

#if 1
    if(st.r_charging) {
      draw_charge_bolt_small(16, 1);
    } else if(st.r_usb) {
      draw_usb_connection_small(16, 1);
    }
#endif
    draw_batt_icon_and_gauge(19, 10, st.r_batt, false);
    draw_batt_percent_small(20, 18, st.r_batt);

    /* 2段目：区切り線 */
    draw_separator(25);

    /* 3段目：Bluetooth */
    char line2[16];
    snprintf(line2, sizeof(line2), "BLE%d", st.ble);
    lv_canvas_draw_text(canvas, 0, 27, CANVAS_W, &dsc, line2);

    /* 4段目：区切り線 */
    draw_separator(38);

    /* 5段目：レイヤ番号 */
    char line4[16];
    snprintf(line4, sizeof(line4), "L%d", st.layer);
    lv_canvas_draw_text(canvas, 0, 40, CANVAS_W, &dsc, line4);

    /* 6段目：レイヤ名 */
    const char *name_full = get_layer_display_name(st.layer);
    char name12[20];
    take_prefix(name_full, name12, sizeof(name12), 12);
    if (name12[0] == '\0') strcpy(name12, "NONE");
    lv_canvas_draw_text(canvas, 0, 50, CANVAS_W, &dsc, name12);

    /* 7段目：区切り線 */
    draw_separator(62);

    /* drawing dot pict */
    const uint8_t *art = g_dimmed ? cat_dim_0 : g_frames[g_frame_idx].data;
    blit_art_32x64_to_canvas(0, 64, art);
}

static void render_from_state(ui_state_t st) {
    build_canvas_from_state(st);
    canvas_to_oled_rot(oled_buf);
    oled_push(oled_buf);
}

/* ===============================
 * Periodic refresh
 * =============================== */
static void refresh_cb(lv_timer_t *t) {
    ARG_UNUSED(t);

    static ui_state_t last = { .layer = -999, .ble = -999,
                               .l_batt = -999, .r_batt = -999,
                               .l_usb = -999, .r_usb = -999,
                               .l_charging = false, .r_charging = false };

    uint32_t now_ms = k_uptime_get_32();
    uint32_t last_act = (uint32_t)atomic_load(&g_last_activity_ms);

    bool active = ((uint32_t)(now_ms - last_act) < ACTIVE_HOLD_MS);
    uint32_t want_period = active ? PERIOD_ACTIVE_MS : PERIOD_IDLE_MS;

    // apply_dim((uint32_t)(now_ms - last_act) >= DIM_AFTER_MS);
    bool dim_now = ((uint32_t)(now_ms - last_act) >= DIM_AFTER_MS);

    if (dim_now != g_dimmed) {
      apply_dim(dim_now);       // ← ここで g_dimmed が更新される想定
      on_dim_changed(dim_now);  // ← 追加
    }

    set_refresh_period(want_period, false);

    bool hinted = atomic_exchange(&g_hint_dirty, false);

    update_central_chg_from_gpio();
    ui_state_t now = read_state();

#if 0
    /* 例：active中かつ非dimの時だけアニメ */
    // bool animate = active && !g_dimmed;
    // maybe_advance_frame(now_ms, animate);
#endif 

    if (hinted || !state_equal(now, last)) {
        bool is_user_action = (now.layer != last.layer) || (now.ble != last.ble);
        if (is_user_action) {
            atomic_store(&g_last_activity_ms, now_ms);
            set_refresh_period(PERIOD_ACTIVE_MS, false);
        }

        last = now;
        render_from_state(now);
#if DEBUG_ENABLE == 1
        LOG_DBG("layer=%d name='%s' ble=%d l_batt=%d r_batt=%d l_charging=%d r_charging=%d",
                now.layer, get_layer_display_name(now.layer),
                now.ble, now.l_batt, now.r_batt, now.l_charging, now.r_charging,
        LOG_DBG("l_usb=%d r_usb=%d active=%d period=%u",
                now.l_usb, now.r_usb
                (int)active, (unsigned)want_period);
#endif
    }
}

static void anim_cb(lv_timer_t *t) {
    ARG_UNUSED(t);

    /* 例：暗いときは止めたいならこの行をON */
    if (g_dimmed) return;

    g_frame_idx = (uint8_t)((g_frame_idx + 1u) % FRAME_COUNT);
    atomic_store(&g_hint_dirty, true);

    /* 変更をすぐ反映したいなら（おすすめ）
       refresh周期が1000msでも“次のtick待ち”にならない */
    if (g_refresh_timer) {
        lv_timer_reset(g_refresh_timer); /* lv_timer_ready が無い環境向け */
    }
}


/* ===============================
 * ZMK hooks
 * =============================== */

static int status_listener(const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh)) {
        mark_activity();
    } else if (as_zmk_battery_state_changed(eh)) {
        mark_dirty_only();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_listener, status_listener);
ZMK_SUBSCRIPTION(status_listener, zmk_layer_state_changed);
//ZMK_SUBSCRIPTION(status_listener, zmk_battery_state_changed);

static int batt_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *br = as_zmk_battery_state_changed(eh);
    if (br) {
        atomic_store(&g_soc_right, (int)br->state_of_charge);

        int now = atomic_load(&g_chg_right);
        if (now == BATT_UNKNOWN) {
          bool usb = usb_connected_guess();
          atomic_store(&g_usb_right, usb);
          atomic_store(&g_chg_right, (usb && br->state_of_charge < 100) ? 1 : 0);
        }

        mark_dirty_only();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_peripheral_battery_state_changed *bl =
        as_zmk_peripheral_battery_state_changed(eh);

    if (bl) {
        int rid = atomic_load(&g_periph_source);
        if (rid < 0) {
            atomic_store(&g_periph_source, (int)bl->source);
            rid = (int)bl->source;
        }

        if (bl->source == PERIPHERAL_SOURCE) {
            atomic_store(&g_soc_left, (int)bl->state_of_charge);

            mark_dirty_only();
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_batt, batt_listener);
ZMK_SUBSCRIPTION(status_batt, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(status_batt, zmk_peripheral_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_INPUT)

static void chg_input_cb(struct input_event *evt) {
    if (evt->type != INPUT_EV_KEY) return;
    if (evt->code != INPUT_KEY_F24) return;   // 右CHG用に割り当てた code

#if DEBUG_ENABLE == 1
    LOG_DBG("PERIPHERAL_CHG evt: type=%d code=%d value=%d", evt->type, evt->code, evt->value);
#endif
    /* value: 1=press, 0=release （ACTIVE_LOWなら “充電中=1” にできる） */
    // g_right_charging = evt->value ? 1 : 0;
    atomic_store(&g_chg_left, evt->value ? 1 : 0);

    mark_dirty_only();
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(chg_input_split)), chg_input_cb);

#endif

/* ===============================
 * screen entry
 * =============================== */
lv_obj_t *zmk_display_status_screen(void) {
    oled_init();

    central_gpio_charge_init();

    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    canvas = lv_canvas_create(screen);
    lv_canvas_set_buffer(canvas, canvas_buf, CANVAS_W, CANVAS_H, LV_IMG_CF_ALPHA_1BIT);

    /* LVGL側で勝手に表示しない（OLEDはdisplay_writeで出す） */
    lv_obj_add_flag(canvas, LV_OBJ_FLAG_HIDDEN);

#if DEBUG_ENABLE == 1
    for (int i = 0; i < LAYER_COUNT; i++) {
        LOG_DBG("names[%d]='%s'", i, layer_display_names[i]);
    }
#endif

    /* 初期状態：idleで開始（操作が来たら即activeへ） */
    uint32_t boot_now = k_uptime_get_32();
    atomic_store(&g_last_activity_ms, boot_now - (ACTIVE_HOLD_MS + 1u));
    atomic_store(&g_timer_period_ms, PERIOD_IDLE_MS);

    atomic_store(&g_hint_dirty, true);
    render_from_state(read_state());

    g_frame_idx = 0;
    g_anim_timer = lv_timer_create(anim_cb, ANIM_PERIOD_MS, NULL);

    /* idle(1Hz)で開始 */
    g_refresh_timer = lv_timer_create(refresh_cb, PERIOD_IDLE_MS, NULL);

    return screen;
}

