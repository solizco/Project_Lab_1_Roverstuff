/*
 * Rover Phase 4 — Competition firmware
 * Board:  Raspberry Pi Pico 2 (RP2350)
 * Camera: Arducam Mega 3MP (SPI) — 128x128 RGB565 red/green/blue ball detection
 * ToF:    2x VL53L0X (FL, FR) + 1x VL53L1X (FM) on shared I2C0
 * Ball:   MG995 arms + TCS3200 color sensor + feeder servo + flywheel
 * Drive:  Custom H-bridge, left/right channels, single-channel wheel encoders
 *
 * State flow:
 *   INIT (arms home + set odometry origin)
 *     --> RUN (search / track / approach / avoid)
 *       --> CAPTURE (FM ToF 108-148 mm + FL/FR clear, 5 consecutive loops)
 *         --> SORT   (identify ball color with the TCS3200)
 *           --> RETURN (encoder odometry: drive back to (0,0), face 0 deg)
 *             --> FIRE   (RED: shoot; GREEN/BLUE: eject to own goal)
 *               --> RECOVER (reverse 500ms)
 *                 --> RUN
 *
 * The start/E-stop button (IDLE and ESTOP states) is supported by the state
 * machine, but the button was never wired: src/button.cpp is a stub, so the
 * rover goes straight from power-on into INIT.
 *
 * Odometry origin is set at INIT wherever the rover is placed.
 * After sorting a ball the rover navigates back to that exact spot,
 * aligns to 0 deg (straight ahead = toward opponent goal), then fires.
 *
 * Pin assignments: see config_pins.h. ToF addresses after XSHUT sequencing:
 *   FL = VL53L0X @ 0x30, FR = VL53L0X @ 0x31, FM = VL53L1X @ 0x32
 */

#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "Arducam_Mega.h"
#include "ArducamCamera.h"

// Pololu-style C++ sensor classes via TwoWire shim
#include "Wire.h"
#include "VL53L0X.h"
#include "VL53L1X.h"

#include "config_pins.h"
#include "button.h"
#include "collector_arms.h"
#include "ball_handler.h"
#include "rover_motors.h"
#include "flywheel_motors.h"
#include "rack_pinion.h"
#include "wheel_encoder.h"
#include "field_boundary.h"

// ============================================================
//  CAMERA VISION — grid density + two-pass cluster
//
//  Detects red, green, and blue balls.
//  When multiple balls are in frame, picks the closest one
//  (densest 8x8 grid cell = most pixels = largest apparent size).
//  Color identification is left to the TCS3200 in SORT state.
// ============================================================
 
#define FRAME_W      128
#define FRAME_H      128
#define FRAME_PIXELS (FRAME_W * FRAME_H)
#define FRAME_BYTES  (FRAME_PIXELS * 2)
 
static Arducam_Mega myCAM(CAM_PIN_CS);
 
// ── Detection filters ─────────────────────────────────────────────────────────
#define MIN_COLOR_PIX      2      // distant ball at 6ft may only be 2-4 pixels
#define MAX_COLOR_PIX    15000      // close-up ball can fill most of 128x128
#define MAX_ASPECT       2.5f     // bbox w/h — balls are round
#define MIN_FILL         0.35f    // colored pix / bbox area
#define CLUSTER_RADIUS   20.0f    // radius=20 isolates one ball at a time
#define MIN_BBOX_DIM      2       // allow 2x2px bbox for distant balls
 
struct VisionResult {
    bool     target_found;
    float    target_x;      // -1.0 (left)  to +1.0 (right)
    float    target_y;      // -1.0 (top)   to +1.0 (bottom)
    uint16_t target_size;   // colored pixel count in cluster
    float    confidence;    // circularity (aspect score 0-1)
    float    fill_ratio;    // colored pix / bbox area
    int16_t  bbox_x0, bbox_y0, bbox_x1, bbox_y1;
};
 
// ── Per-pixel color test ──────────────────────────────────────────────────────
// Channel difference thresholds from real Arducam readings under ECE 101 lights:
//   Red:   R=216 G=48  B=48  → diff=168
//   Green: R=96  G=216 B=136 → diff=120
//   Blue:  R=16  G=88  B=192 → diff=104
// Red uses stricter g<80 && b<80 ceiling to reject skin (g≈120) and brown.
static inline bool rgb565_is_ball_color(uint8_t hi, uint8_t lo) {
    uint8_t r = (hi & 0xF8);
    uint8_t g = ((hi & 0x07) << 5) | ((lo & 0xE0) >> 3);
    uint8_t b = (lo & 0x1F) << 3;
 
    uint8_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    if (mx < 40) return false;
 
    // Red: strict ceiling rejects skin (g≈120) and brown clothing
    if (r >= 70 && (int)r - (int)g > 35 && (int)r - (int)b > 35
        && g < 100 && b < 100) return true;
 
    // Green dominant
    if (g >= 70 && (int)g - (int)r > 35 && (int)g - (int)b > 35) return true;
 
    // Blue dominant
    if (b >= 70 && (int)b - (int)r > 35 && (int)b - (int)g > 25) return true;
 
    return false;
}
 
// ── Grid density + two-pass cluster ──────────────────────────────────────────
static VisionResult process_frame_blob(const uint8_t* frame_data) {
    VisionResult result = {false, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0, 0, 0, 0};

    // ── Pass 1: grid density ──────────────────────────────────────────────────
    // 8px cells → 16x16 grid. Distant 2-3px balls land in one cell.
    // Densest cell = closest ball. Works correctly with 12 balls in frame.
    static const int CELL  = 8;
    static const int GCOLS = FRAME_W / CELL;   // 16
    static const int GROWS = FRAME_H / CELL;   // 16
    uint8_t grid[GROWS][GCOLS];
    memset(grid, 0, sizeof(grid));

    for (int y = 0; y < FRAME_H; y++) {
        for (int x = 0; x < FRAME_W; x++) {
            int i = y * FRAME_W + x;
            if (!rgb565_is_ball_color(frame_data[i*2], frame_data[i*2+1])) continue;
            int gy = y / CELL;
            int gx = x / CELL;
            if (grid[gy][gx] < 255) grid[gy][gx]++;
        }
    }

    int best_count = 0, best_gy = 0, best_gx = 0;
    for (int gy = 0; gy < GROWS; gy++) {
        for (int gx = 0; gx < GCOLS; gx++) {
            if (grid[gy][gx] > best_count) {
                best_count = grid[gy][gx];
                best_gy    = gy;
                best_gx    = gx;
            }
        }
    }

    // Too few pixels — nothing detected
    if (best_count < 2) return result;
    // Too dense — large colored object (shirt, wall, equipment), not a ball
    if (best_count > 200) return result;
    // Top 40% of frame is background — balls sit on the floor
    if (best_gy < (GROWS * 40 / 100)) return result;

    float cx = (best_gx + 0.5f) * CELL;
    float cy = (best_gy + 0.5f) * CELL;

    // ── Pass 2: cluster within CLUSTER_RADIUS ────────────────────────────────
    // Radius=20 keeps each ball isolated — two balls 50px apart won't merge.
    const float R2 = CLUSTER_RADIUS * CLUSTER_RADIUS;
    int     x_min = FRAME_W, y_min = FRAME_H;
    int     x_max = -1,      y_max = -1;
    int32_t cx2   = 0,       cy2   = 0;
    int     pix2  = 0;

    for (int y = 0; y < FRAME_H; y++) {
        float dy = (float)y - cy;
        if (dy * dy > R2) continue;
        for (int x = 0; x < FRAME_W; x++) {
            float dx = (float)x - cx;
            if (dx * dx + dy * dy > R2) continue;
            int i = y * FRAME_W + x;
            if (!rgb565_is_ball_color(frame_data[i*2], frame_data[i*2+1])) continue;
            pix2++;
            cx2 += x; cy2 += y;
            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
    }

    if (pix2 < MIN_COLOR_PIX) return result;
    if (pix2 > MAX_COLOR_PIX) return result;

    int   w    = x_max - x_min + 1;
    int   h    = y_max - y_min + 1;

    if (w < MIN_BBOX_DIM || h < MIN_BBOX_DIM) return result;

    float asp  = (w > h) ? (float)w / (float)h : (float)h / (float)w;
    float fill = (float)pix2 / (float)(w * h);

    // Tightened from 2.5 — balls are round, non-ball objects tend to be elongated
    if (asp  > MAX_ASPECT)   return result;
    if (fill < MIN_FILL)     return result;

    float fcx = (float)cx2 / (float)pix2;
    float fcy = (float)cy2 / (float)pix2;

    result.target_found = true;
    result.target_x     = (fcx - FRAME_W * 0.5f) / (FRAME_W * 0.5f);
    result.target_y     = (fcy - FRAME_H * 0.5f) / (FRAME_H * 0.5f);
    result.target_size  = (uint16_t)pix2;
    result.confidence   = (w >= h) ? (float)h / (float)w : (float)w / (float)h;
    result.fill_ratio   = fill;
    result.bbox_x0      = (int16_t)x_min;
    result.bbox_y0      = (int16_t)y_min;
    result.bbox_x1      = (int16_t)x_max;
    result.bbox_y1      = (int16_t)y_max;

    return result;
}
 
static void read_camera_frame(ArducamCamera* cam,
                               uint8_t*       frame_buf,
                               uint8_t*       read_chunk) {
    myCAM.takePicture((CAM_IMAGE_MODE)CAM_IMAGE_MODE_128X128,
                      (CAM_IMAGE_PIX_FMT)CAM_IMAGE_PIX_FMT_RGB565);
    uint32_t total     = myCAM.getTotalLength();
    uint32_t bytesRead = 0;
 
    while (bytesRead < total && bytesRead < FRAME_BYTES) {
        uint32_t remaining = total - bytesRead;
        uint8_t  chunk     = (remaining > 200) ? 200 : (uint8_t)remaining;
        readBuff(cam, read_chunk, chunk);
        uint32_t copyLen = chunk;
        if (bytesRead + copyLen > FRAME_BYTES) copyLen = FRAME_BYTES - bytesRead;
        memcpy(&frame_buf[bytesRead], read_chunk, copyLen);
        bytesRead += chunk;
    }
    while (bytesRead < total) {
        uint32_t remaining = total - bytesRead;
        uint8_t  chunk     = (remaining > 200) ? 200 : (uint8_t)remaining;
        readBuff(cam, read_chunk, chunk);
        bytesRead += chunk;
    }
}

// ============================================================
//  TOF COLLISION AVOIDANCE — mixed L0X (FL/FR) + L1X (FM)
//
//  Pololu VL53L0X / VL53L1X communicate through a TwoWire shim.
//  One TwoWire instance wraps the shared I2C0 bus.
//  Key API mapping vs old free-function style:
//    sensor.setBus(&Wire0)       → replaces i2c port registration
//    sensor.setAddress(addr)     → replaces *_set_address()
//    sensor.init()               → replaces *_init() / SensorInit()
//    sensor.startContinuous(ms)  → replaces StartRanging()
//    sensor.read(false) / sensor.readRangeContinuousMillimeters()
//                                → replaces CheckForDataReady + GetDistance
// ============================================================

#define NUM_SENSORS       3
#define VL53_DEFAULT_ADDR 0x29

// Final reassigned addresses — must be unique and != 0x29
static const uint8_t SENSOR_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32};

// XSHUT GPIO pins — held LOW to keep sensor in hardware reset
static const uint8_t XSHUT_PINS[NUM_SENSORS] = {
    TOF_PIN_XSHUT_FL,   // FL  VL53L0X
    TOF_PIN_XSHUT_FR,   // FR  VL53L0X
    TOF_PIN_XSHUT_FM    // FM  VL53L1X
};

#define STOP_DISTANCE_MM 85
#define SLOW_DISTANCE_MM 175

typedef enum { ZONE_STOP, ZONE_SLOW, ZONE_CLEAR, ZONE_ERROR } DistanceZone;
static const char* zone_names[] = {"STOP", "SLOW", "CLR ", "ERR "};

#define MEDIAN_WINDOW 5

typedef struct {
    uint16_t buffer[MEDIAN_WINDOW];
    uint8_t  index;
    uint8_t  count;
} MedianFilter;

static void mf_init(MedianFilter* f) { memset(f, 0, sizeof(MedianFilter)); }

static void mf_add(MedianFilter* f, uint16_t val) {
    f->buffer[f->index] = val;
    f->index = (f->index + 1) % MEDIAN_WINDOW;
    if (f->count < MEDIAN_WINDOW) f->count++;
}

static uint16_t mf_get(MedianFilter* f) {
    if (f->count == 0) return 0;
    uint16_t sorted[MEDIAN_WINDOW];
    memcpy(sorted, f->buffer, sizeof(uint16_t) * f->count);
    // Insertion sort — at most MEDIAN_WINDOW elements
    for (uint8_t i = 1; i < f->count; i++) {
        uint16_t v = sorted[i];
        int8_t   j = (int8_t)i - 1;
        while (j >= 0 && sorted[j] > v) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = v;
    }
    return sorted[f->count / 2];
}

typedef struct {
    uint16_t     filtered_mm;
    DistanceZone zone;
    bool         initialized;
    MedianFilter filter;
    uint8_t stale_count;
} ToFSensor;

static ToFSensor tof[NUM_SENSORS];

// One TwoWire shim for the shared I2C bus
static TwoWire Wire0(TOF_I2C);

// Sensor objects — two L0X (FL, FR) and one L1X (FM)
static VL53L0X sensorFL;
static VL53L0X sensorFR;
static VL53L1X sensorFM;

static DistanceZone classify(uint16_t mm) {
    if (mm == 0)               return ZONE_ERROR;
    if (mm < STOP_DISTANCE_MM) return ZONE_STOP;
    if (mm < SLOW_DISTANCE_MM) return ZONE_SLOW;
    return ZONE_CLEAR;
}

// FM ToF thresholds for ball capture confirmation
#define FM_CAPTURE_MIN_MM  108   // ball must be at least this close
#define FM_CAPTURE_MAX_MM  148  // ball must be no further than this
#define SIDE_CLEAR_MM      190  // FL/FR must exceed this to rule out a rover

struct CollisionResult {
    bool         danger_left;
    bool         danger_right;
    bool         front_blocked;
    bool         ball_in_capture_zone;  // FM in range AND FL/FR clear
    DistanceZone fl_zone, fr_zone, fm_zone;
    uint16_t     fl_mm,  fr_mm,  fm_mm;
};

static bool init_all_tof() {
    int ok = 0;

    // Hold all sensors in reset before starting sequence
    gpio_put(TOF_PIN_XSHUT_FL, 0);
    gpio_put(TOF_PIN_XSHUT_FR, 0);
    gpio_put(TOF_PIN_XSHUT_FM, 0);
    sleep_ms(20);

    // ── FL: VL53L0X ──────────────────────────────────────────────────────────
    gpio_put(TOF_PIN_XSHUT_FL, 1);
    sleep_ms(100);
    sensorFL.setBus(&Wire0);
    sensorFL.setTimeout(500);
    if (!sensorFL.init()) {
        printf("[TOF] FL: init() failed\n");
        stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FL, 0);
    } else {
        sensorFL.setAddress(SENSOR_ADDRS[0]);
        sleep_ms(20);
        tof[0].initialized = true;
        mf_init(&tof[0].filter);
        printf("[TOF] FL: OK at 0x%02X\n", SENSOR_ADDRS[0]);
        stdio_flush();
        ok++;
    }

    // ── FR: VL53L0X ──────────────────────────────────────────────────────────
    gpio_put(TOF_PIN_XSHUT_FR, 1);
    sleep_ms(100);
    sensorFR.setBus(&Wire0);
    sensorFR.setTimeout(500);
    if (!sensorFR.init()) {
        printf("[TOF] FR: init() failed\n");
        stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FR, 0);
    } else {
        sensorFR.setAddress(SENSOR_ADDRS[1]);
        sleep_ms(20);
        tof[1].initialized = true;
        mf_init(&tof[1].filter);
        printf("[TOF] FR: OK at 0x%02X\n", SENSOR_ADDRS[1]);
        stdio_flush();
        ok++;
    }

    // ── FM: VL53L1X ──────────────────────────────────────────────────────────
    gpio_put(TOF_PIN_XSHUT_FM, 1);
    sleep_ms(100);
    sensorFM.setBus(&Wire0);
    sensorFM.setTimeout(500);
    if (!sensorFM.init()) {
        printf("[TOF] FM: init() failed\n");
        stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FM, 0);
    } else {
        sensorFM.setAddress(SENSOR_ADDRS[2]);
        sleep_ms(20);
        sensorFM.setDistanceMode(VL53L1X::Short);
        sensorFM.setMeasurementTimingBudget(50000);  // 50 ms
        tof[2].initialized = true;
        mf_init(&tof[2].filter);
        printf("[TOF] FM: OK at 0x%02X\n", SENSOR_ADDRS[2]);
        stdio_flush();
        ok++;
    }

    // Start continuous ranging AFTER all address assignments are complete.
    // Starting any sensor early causes I2C traffic that corrupts SPAD
    // calibration reads on sensors still being initialized.
    printf("[TOF] Starting continuous ranging...\n");
    stdio_flush();
    if (tof[0].initialized) sensorFL.startContinuous(0);
    if (tof[1].initialized) sensorFR.startContinuous(0);
    if (tof[2].initialized) sensorFM.startContinuous(55);

    printf("[TOF] %d/3 sensors ready\n", ok);
    stdio_flush();
    return (ok >= 2);
}

static void read_tof() {
    // ── FL (VL53L0X) — blocking poll ─────────────────────────────────────────
    if (tof[0].initialized) {
        uint16_t d       = sensorFL.readRangeContinuousMillimeters();
        bool     timeout = sensorFL.timeoutOccurred();

        if (!timeout && d >= 8000) {
            // VL53L0X returns 8191 when no target is in range — this is normal
            // for an outward-angled sensor pointing at open space. Treat as CLEAR.
            tof[0].stale_count = 0;
            tof[0].filtered_mm = 2000;
        } else if (!timeout && d > 0 && d < 8000) {
            // Good read — reset stale counter and push into filter
            tof[0].stale_count = 0;
            mf_add(&tof[0].filter, d);
            tof[0].filtered_mm = mf_get(&tof[0].filter);
        } else {
            // Actual bad read (timeout or d==0) — increment stale counter
            tof[0].stale_count++;
            if (tof[0].stale_count > 5) {
                // Sensor unresponsive for 5+ loops (~150ms at 30ms loop rate).
                // Flush the filter so a stale STOP value doesn't freeze the rover.
                // Default to 0 which classifies as ZONE_ERROR — not ZONE_STOP —
                // so the rover continues rather than reversing indefinitely.
                printf("[TOF] FL: stale — flushing filter\n");
                stdio_flush();
                mf_init(&tof[0].filter);
                tof[0].filtered_mm = 0;
                tof[0].stale_count = 0;
            }
        }
        tof[0].zone = classify(tof[0].filtered_mm);
    } else {
        tof[0].zone = ZONE_ERROR;
    }

    // ── FR (VL53L0X) — blocking poll ─────────────────────────────────────────
    if (tof[1].initialized) {
        uint16_t d       = sensorFR.readRangeContinuousMillimeters();
        bool     timeout = sensorFR.timeoutOccurred();
        if (!timeout && d >= 8000) {
            // No target in range — same handling as FL: treat as CLEAR
            tof[1].stale_count = 0;
            tof[1].filtered_mm = 2000;
        } else if (!timeout && d > 0 && d < 8000) {
            tof[1].stale_count = 0;
            mf_add(&tof[1].filter, d);
            tof[1].filtered_mm = mf_get(&tof[1].filter);
        } else {
            tof[1].stale_count++;
            if (tof[1].stale_count > 5) {
                printf("[TOF] FR: stale — flushing filter\n");
                stdio_flush();
                mf_init(&tof[1].filter);
                tof[1].filtered_mm = 0;
                tof[1].stale_count = 0;
            }
        }
        tof[1].zone = classify(tof[1].filtered_mm);
    } else {
        tof[1].zone = ZONE_ERROR;
    }
 
    // ── FM (VL53L1X) — non-blocking poll ─────────────────────────────────────
    // dataReady() reads the interrupt flag. read(false) fetches the measurement
    // and clears the interrupt without blocking.
    // For FM we use a longer stale threshold (10 loops) because dataReady()
    // legitimately returns false between 55ms measurement periods — that is
    // not a fault, just normal timing. We only flush if we get actual bad
    // reads, not just "not ready yet".
     if (tof[2].initialized) {
        if (sensorFM.dataReady()) {
            uint16_t d = sensorFM.read(false);
            if (!sensorFM.timeoutOccurred() && d >= 8000) {
                // No target in range — treat as max range, not error
                tof[2].stale_count = 0;
                tof[2].filtered_mm = 2000;
            } else if (!sensorFM.timeoutOccurred() && d > 0 && d < 8000) {
                tof[2].stale_count = 0;
                mf_add(&tof[2].filter, d);
                tof[2].filtered_mm = mf_get(&tof[2].filter);
            } else {
                tof[2].stale_count++;
                if (tof[2].stale_count > 10) {
                    printf("[TOF] FM: stale — flushing filter\n");
                    stdio_flush();
                    mf_init(&tof[2].filter);
                    tof[2].filtered_mm = 2000;  // default to clear not error
                    tof[2].stale_count = 0;
                }
            }
        }
        tof[2].zone = classify(tof[2].filtered_mm);
    }
}
static CollisionResult get_collision() {
    CollisionResult c;
    c.fl_mm   = tof[0].filtered_mm;
    c.fr_mm   = tof[1].filtered_mm;
    c.fm_mm   = tof[2].filtered_mm;
    c.fl_zone = tof[0].zone;
    c.fr_zone = tof[1].zone;
    c.fm_zone = tof[2].zone;

    // FL/FR at 45 degrees — only used for side rover detection
    c.danger_left  = (tof[0].zone == ZONE_STOP);
    c.danger_right = (tof[1].zone == ZONE_STOP);

    // FM is forward-facing — use it for front obstacle detection only.
    // Exclude capture zone so FM detecting a ball doesn't trigger reverse.
    bool fm_in_range = (c.fm_mm >= FM_CAPTURE_MIN_MM && c.fm_mm <= FM_CAPTURE_MAX_MM);
    bool sides_clear = (c.fl_mm == 0 || c.fl_mm > SIDE_CLEAR_MM) &&
                       (c.fr_mm == 0 || c.fr_mm > SIDE_CLEAR_MM);
    c.ball_in_capture_zone = fm_in_range && sides_clear;

    // Front blocked = FM sees something close that is NOT a ball to capture
    c.front_blocked = (c.fm_mm > 0 && c.fm_mm < STOP_DISTANCE_MM && !c.ball_in_capture_zone);

    return c;
}

// ============================================================
//  DRIVE COMMAND & FUSION
// ============================================================

typedef enum {
    MODE_SEARCH,
    MODE_TRACK,
    MODE_APPROACH,
    MODE_AVOID,
    MODE_REVERSE,
    MODE_STOP
} DriveMode;

static const char* mode_names[] = {"SEARCH", "TRACK ", "APPRCH", "AVOID ", "REVRSE", "STOP  "};

struct DriveCommand {
    float     linear_speed;
    float     turn_rate;
    DriveMode mode;
};

static const float SPEED_FULL    =  1.0f;
static const float SPEED_SLOW    =  0.8f;
static const float SPEED_REVERSE = -0.8f;
static const float TURN_TRACK    =  0.3f;
static const float TURN_AVOID    =  0.3f;
static const float TURN_SEARCH   =  0.72f;

static uint32_t  target_lost_count   = 0;
static float     last_target_x       = 0.0f;
static uint16_t  last_target_size    = 0;
static uint8_t   approach_hold_count = 0;
static float     last_approach_turn  = 0.0f;


static int      capture_confirm_count = 0;
static uint32_t search_forward_until  = 0;
static uint32_t search_cycle_count    = 0;


static DriveCommand compute_drive(const VisionResult&    vision,
                                  const CollisionResult& collision) {
    DriveCommand cmd = {0.0f, 0.0f, MODE_STOP};

    // Front blocked — reverse away, steer toward the more open side
    if (collision.front_blocked) {
        cmd.mode         = MODE_REVERSE;
        cmd.linear_speed = SPEED_REVERSE;
        cmd.turn_rate    = (collision.fl_mm < collision.fr_mm)
                           ? TURN_AVOID : -TURN_AVOID;
        target_lost_count = 0;
        return cmd;
    }

    // Side obstacle avoidance
     if (collision.danger_left && !collision.danger_right) {
        cmd.mode         = MODE_REVERSE;
        cmd.linear_speed = SPEED_REVERSE;
        cmd.turn_rate    = TURN_AVOID;  // both wheels reverse, right faster = pivots left away from obstacle
        return cmd;
    }
    if (collision.danger_right && !collision.danger_left) {
        cmd.mode         = MODE_REVERSE;
        cmd.linear_speed = SPEED_REVERSE;
        cmd.turn_rate    = -TURN_AVOID;   // both wheels reverse, left faster = pivots right away from obstacle
        return cmd;
    }

   // ── Camera tracking ───────────────────────────────────────────────────────

    // Approach hold: if we just lost the ball mid-approach, coast briefly on
    // last known heading instead of immediately falling into search/spin.
    if (!vision.target_found && approach_hold_count > 0) {
        approach_hold_count--;
        cmd.mode         = MODE_APPROACH;
        cmd.linear_speed = 0.72f;
        cmd.turn_rate    = last_approach_turn * 0.5f;
        return cmd;
    }

    if (vision.target_found) {

        // ── Spin exit deceleration ────────────────────────────────────────────
        // Coming out of SPIN (lost >= 10): bleed angular momentum for 1 frame
        // before committing to full track. Prevents the lurch on reacquisition.
        if (target_lost_count >= 10) {
            printf("[VISION] reacquired from SPIN — decel frame sz=%u x=%.2f\n",
                vision.target_size, vision.target_x);
            stdio_flush();
            target_lost_count   = 0;
            approach_hold_count = 0;
            cmd.mode            = MODE_TRACK;
            cmd.linear_speed    = 0.72f;
            cmd.turn_rate       = vision.target_x * TURN_TRACK * 0.35f;
            last_approach_turn  = cmd.turn_rate;
            return cmd;
        }

        if (target_lost_count > 0) {
            printf("[VISION] ball found — resetting lost count from %lu sz=%u x=%.2f\n",
                (unsigned long)target_lost_count, vision.target_size, vision.target_x);
            stdio_flush();
        }

        target_lost_count   = 0;
        approach_hold_count = 0;
        last_target_x       = vision.target_x;
        last_target_size    = vision.target_size;

        if (vision.target_size >= 20) {

            // ── APPROACH mode ─────────────────────────────────────────────────
            // Turn rate scales down as ball grows larger (closer) to prevent
            // overshoot. At sz=25 → full TURN_TRACK. At sz=425+ → 30% floor.
            float approach_scale = 1.0f - ((float)(vision.target_size - 15) / 400.0f);
            if (approach_scale < 0.3f) approach_scale = 0.3f;

            cmd.mode      = MODE_APPROACH;
            cmd.turn_rate = vision.target_x * TURN_TRACK * approach_scale;

            // Save turn rate AFTER computing it so last_approach_turn is valid
            last_approach_turn  = cmd.turn_rate;
            approach_hold_count = 4;

            if (collision.ball_in_capture_zone) {
                // Ball confirmed under arms — stop dead
                cmd.linear_speed = 0.0f;
                cmd.turn_rate    = 0.0f;

            } else if (tof[2].filtered_mm > FM_CAPTURE_MAX_MM + 200) {
                // Far approach — moderate speed, camera steering
                cmd.linear_speed = 0.78f;

            } else if (tof[2].filtered_mm > FM_CAPTURE_MAX_MM + 100) {
                // Getting close — ease off
                cmd.linear_speed = 0.74f;

            } else if (tof[2].filtered_mm > FM_CAPTURE_MAX_MM) {
                // Just outside zone — minimum moving speed, very gentle steering
                cmd.linear_speed = 0.72f;
                cmd.turn_rate    = vision.target_x * TURN_TRACK * approach_scale * 0.3f;

            } else if (tof[2].filtered_mm > (FM_CAPTURE_MIN_MM + FM_CAPTURE_MAX_MM) / 2) {
                // Upper half of capture zone — tiny nudge forward only, no steering
                cmd.linear_speed = 0.72f;
                cmd.turn_rate    = 0.0f;

            } else {
                // Mid-zone or closer — hold completely still
                cmd.linear_speed = 0.0f;
                cmd.turn_rate    = 0.0f;
            }

        } else {
            // ── TRACK mode — ball still far, target_size < 20 ────────────────
            cmd.mode      = MODE_TRACK;
            cmd.turn_rate = vision.target_x * TURN_TRACK;

            // Cap track speed so the transition to approach isn't a lurch
            if (tof[2].zone == ZONE_SLOW && !collision.ball_in_capture_zone)
                cmd.linear_speed = 0.75f;
            else if (vision.target_size > 8)
                cmd.linear_speed = 0.82f;   // ball visible and moderate size — ease off
            else
                cmd.linear_speed = SPEED_FULL;
        }

        return cmd;
    }

     // ── Ball lost — search ────────────────────────────────────────────────────
    target_lost_count++;
    cmd.mode = MODE_SEARCH;
    printf("[SEARCH] lost=%lu phase=%s last_x=%.2f last_sz=%u fwd_until=%s\n",
        (unsigned long)target_lost_count,
        target_lost_count < 10 ? "COAST" : "SPIN",
        last_target_x,
        last_target_size,
        time_us_32() < search_forward_until ? "ACTIVE" : "idle");
    stdio_flush();

    // Forward burst — repositions rover after spin cycle
    if (time_us_32() < search_forward_until) {
        cmd.linear_speed = 0.75f;
        cmd.turn_rate    = 0.0f;
        return cmd;
    }

    if (target_lost_count < 10) {
        // ── COAST phase — ball was just visible, drive toward last known direction
        // Scale speed down if ball was large (close) when lost — don't overshoot
        float coast_speed = (last_target_size >= 100) ? 0.72f : 0.82f;
        cmd.linear_speed = coast_speed;
        // Stronger last-direction bias than before (0.25 vs 0.15)
        cmd.turn_rate    = last_target_x * 0.25f;
    } else {
        // ── SPIN phase — ball not found after coasting, rotate to scan
        cmd.linear_speed = 0.0f;
        cmd.turn_rate    = TURN_SEARCH;

        if (target_lost_count % 480 == 0) {
            search_cycle_count++;
            if (search_cycle_count >= 2 ) {
                search_cycle_count   = 0;
                search_forward_until = time_us_32() + 2000000u;
                target_lost_count    = 0;
            }
        }
    }

    return cmd;
}
// ============================================================
//  ODOMETRY-BASED NAVIGATION
//
//  All motor control goes through motors_set() — the same path
//  used during RUN. This guarantees the same PWM configuration,
//  MIN_DUTY clamping, and H-bridge behavior that is already
//  proven to move the rover.
//
//  nav_rotate_to() and nav_return_to_origin() keep
//  enc.set_left/right_dir() in sync so odometry stays valid
//  during the blocking navigation sequence.
// ============================================================
 
// ── Nav motor primitives ──────────────────────────────────────────────────────
// These are thin wrappers around motors_set() — no separate PWM config needed.
 
static WheelEncoder enc;
static FieldBoundary boundary;

// ── Nav constants ─────────────────────────────────────────────────────────────
static const float NAV_DRIVE_SPEED = 0.85f;
static const float NAV_TURN_SPEED  = 0.82f;
static const float NAV_TRIM_KP     = 0.006f;
static const float NAV_ANGLE_TOL   = 0.06f;
static const float NAV_DIST_TOL    = 25.0f;
static const float NAV_HEADING_TOL = 0.12f;
static const uint32_t NAV_ROTATE_TIMEOUT_US = 5000000u;   // give up on a turn after 5 s
static const uint32_t NAV_RETURN_TIMEOUT_US = 20000000u;  // give up on RETURN after 20 s

static void _nav_stop() {
    motors_stop();
    enc.set_left_dir(0);
    enc.set_right_dir(0);
}

static void nav_rotate_to(float target_theta) {
    const uint32_t start_us = time_us_32();
    while (true) {
        enc.update_odometry();
        float err = WheelEncoder::normalize_angle(target_theta - enc.theta());
        if (fabsf(err) < NAV_ANGLE_TOL) break;
        if ((time_us_32() - start_us) > NAV_ROTATE_TIMEOUT_US) {
            printf("[NAV] Rotate timeout — err=%.1fdeg\n", err * 180.0f / (float)M_PI);
            stdio_flush();
            break;
        }
        if (err > 0.0f) {
            enc.set_left_dir(-1); enc.set_right_dir(1);
            motors_set(-NAV_TURN_SPEED, NAV_TURN_SPEED);
        } else {
            enc.set_left_dir(1); enc.set_right_dir(-1);
            motors_set(NAV_TURN_SPEED, -NAV_TURN_SPEED);
        }
        sleep_ms(20);
    }
    _nav_stop();
    sleep_ms(200);
}

static void nav_return_to_origin() {
    float tx = 0.0f;   // always return to placement position
    float ty = 0.0f;

    printf("[NAV] Returning to (%.0f, %.0f)\n", tx, ty);
    printf("[NAV] current=(%.1f,%.1f) theta=%.1fdeg dist=%.1fmm\n",
           enc.x(), enc.y(),
           enc.theta() * 180.0f / (float)M_PI,
           sqrtf((enc.x()-tx)*(enc.x()-tx) + (enc.y()-ty)*(enc.y()-ty)));
    stdio_flush();

    // Step 1: rotate to face origin
    nav_rotate_to(atan2f(ty - enc.y(), tx - enc.x()));

    // Step 2: drive with tick-error trim + heading correction
    int32_t start_left  = enc.left_ticks();
    int32_t start_right = enc.right_ticks();
    float   last_dist   = sqrtf((enc.x()-tx)*(enc.x()-tx) +
                                 (enc.y()-ty)*(enc.y()-ty));
    int     stuck_count = 0;

    enc.set_left_dir(1); enc.set_right_dir(1);
    motors_set(NAV_DRIVE_SPEED, NAV_DRIVE_SPEED);
    const uint32_t return_start_us = time_us_32();

    while (true) {
        enc.update_odometry();
        float dx   = tx - enc.x();
        float dy   = ty - enc.y();
        float dist = sqrtf(dx*dx + dy*dy);

        if (dist <= NAV_DIST_TOL) break;
        if ((time_us_32() - return_start_us) > NAV_RETURN_TIMEOUT_US) {
            printf("[NAV] Return timeout — %.0fmm from origin\n", dist);
            stdio_flush();
            break;
        }

        if (fabsf(last_dist - dist) < 2.0f) stuck_count++;
        else { stuck_count = 0; last_dist = dist; }
        if (stuck_count > 50) {
            printf("[NAV] Stuck — stopping\n"); stdio_flush();
            break;
        }

        float heading_err = WheelEncoder::normalize_angle(
                                atan2f(dy, dx) - enc.theta());
        float spd = (dist < 150.0f) ? 0.80f : NAV_DRIVE_SPEED;

        if (fabsf(heading_err) > NAV_HEADING_TOL) {
            _nav_stop();
            nav_rotate_to(atan2f(ty - enc.y(), tx - enc.x()));
            start_left  = enc.left_ticks();
            start_right = enc.right_ticks();
            enc.set_left_dir(1); enc.set_right_dir(1);
            motors_set(spd, spd);
        } else {
            int32_t dl         = enc.left_ticks()  - start_left;
            int32_t dr         = enc.right_ticks() - start_right;
            float   correction = (float)(dr - dl) * NAV_TRIM_KP;
            float   l = spd + correction;
            float   r = spd - correction;
            if (l >  1.0f) l =  1.0f;
            if (l < -1.0f) l = -1.0f;
            if (r >  1.0f) r =  1.0f;
            if (r < -1.0f) r = -1.0f;
            enc.set_left_dir(1); enc.set_right_dir(1);
            motors_set(l, r);
        }
        sleep_ms(20);
    }

    // Step 3: face opponent (theta=0)
    _nav_stop();
    sleep_ms(200);
    nav_rotate_to(0.0f);

    printf("[NAV] Done. pos=(%.1f,%.1f) theta=%.1fdeg dist=%.1fmm\n",
           enc.x(), enc.y(),
           enc.theta() * 180.0f / (float)M_PI,
           sqrtf((enc.x()-tx)*(enc.x()-tx) + (enc.y()-ty)*(enc.y()-ty)));
    stdio_flush();
}
 

// ============================================================
//  TOP-LEVEL STATE MACHINE
// ============================================================

enum RoverState {
    STATE_IDLE    = 0,
    STATE_INIT,
    STATE_RUN,
    STATE_CAPTURE,
    STATE_SORT,
    STATE_RETURN,   // NEW: navigate back to (0,0)
    STATE_FIRE,     // NEW: shoot from origin facing 0 deg
    STATE_RECOVER,
    STATE_ESTOP,
};

static const char* state_name(RoverState s) {
    switch (s) {
        case STATE_IDLE:    return "IDLE";
        case STATE_INIT:    return "INIT";
        case STATE_RUN:     return "RUN";
        case STATE_CAPTURE: return "CAPTURE";
        case STATE_SORT:    return "SORT";
        case STATE_RETURN:  return "RETURN";
        case STATE_FIRE:    return "FIRE";
        case STATE_RECOVER: return "RECOVER";
        case STATE_ESTOP:   return "ESTOP";
    }
    return "?";
}

static const uint32_t RECOVER_DRIVE_MS = 500;
static const float    RECOVER_SPEED    = -0.40f;

// Motor test constants — run once during INIT to verify all wheels spin
static const float    MOTOR_TEST_DUTY  =  0.8f;
static const uint32_t MOTOR_TEST_MS    = 8000;

// ── Apply a DriveCommand to the motors ───────────────────────────────────────
// Also updates encoder direction so odometry stays valid during RUN.
static void apply_drive_command(const DriveCommand& cmd) {
    float left  = cmd.linear_speed + cmd.turn_rate;
    float right = cmd.linear_speed - cmd.turn_rate;
    if (left  >  1.25f) left  =  1.25f;
    if (left  < -1.25f) left  = -1.25f;
    if (right >  1.25f) right =  1.25f;
    if (right < -1.25f) right = -1.25f;

    printf("[DRIVE] L=%.2f R=%.2f\n", left, right);
    stdio_flush();

    // Keep encoder direction in sync so pose updates are signed correctly
    enc.set_left_dir (left  > 0.01f ? 1 : left  < -0.01f ? -1 : 0);
    enc.set_right_dir(right > 0.01f ? 1 : right < -0.01f ? -1 : 0);

    motors_set(left, right);
}

static void all_stop() {
    motors_stop();
    flywheels_stop();
    enc.set_left_dir(0);
    enc.set_right_dir(0);
}

static void enter_estop() {
    all_stop();
    arms_release();
    feeder_release();
    gpio_put(LED_PIN, 0);
    printf("[STATE] *** E-STOP ***\n");
    stdio_flush();
}

static void blink_n(int n, uint32_t ms) {
    for (int i = 0; i < n; i++) {
        gpio_put(LED_PIN, 1); sleep_ms(ms);
        gpio_put(LED_PIN, 0); sleep_ms(ms);
    }
}

// ============================================================
//  MAIN
// ============================================================

int main() {

    // ── Arm servos: PWM at home position FIRST — before anything else ─────────
    // Do NOT gpio_init/gpio_put LOW first — a constant LOW = no signal = servo drifts.
    // Setting PWM output immediately gives the servo a valid position command
    // from the first millisecond of power-on, preventing boot jerk.
    gpio_set_function(ARM_PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(ARM_PIN_B, GPIO_FUNC_PWM);
    {
        uint arm_slice = pwm_gpio_to_slice_num(ARM_PIN_A);
        pwm_set_clkdiv(arm_slice, 150.0f);
        pwm_set_wrap(arm_slice, 20000);
        pwm_set_chan_level(arm_slice, pwm_gpio_to_channel(ARM_PIN_A),
                           600 + (2400 - 600) * (ARMS_ROM_MAX + ARM_A_TRIM_TOP_DEG) / 180);  // arm A includes its top-position trim
        pwm_set_chan_level(arm_slice, pwm_gpio_to_channel(ARM_PIN_B),
                           600 + (2400 - 600) * (180 - ARMS_ROM_MAX) / 180);
        pwm_set_enabled(arm_slice, true);
    }

    // Pull all motor pins LOW immediately — prevents float-driven spin at boot.
    // This must happen before stdio_init_all / USB wait.
    gpio_init(MOTOR_L_PIN_IN3); gpio_set_dir(MOTOR_L_PIN_IN3, GPIO_OUT); gpio_put(MOTOR_L_PIN_IN3, 0);
    gpio_init(MOTOR_L_PIN_IN4); gpio_set_dir(MOTOR_L_PIN_IN4, GPIO_OUT); gpio_put(MOTOR_L_PIN_IN4, 0);
    gpio_init(MOTOR_R_PIN_IN1); gpio_set_dir(MOTOR_R_PIN_IN1, GPIO_OUT); gpio_put(MOTOR_R_PIN_IN1, 0);
    gpio_init(MOTOR_R_PIN_IN2); gpio_set_dir(MOTOR_R_PIN_IN2, GPIO_OUT); gpio_put(MOTOR_R_PIN_IN2, 0);

    // Pull all ToF XSHUT pins LOW immediately. Both VL53L0X and VL53L1X have
    // an internal pull-up on XSHUT — if left floating they boot together at
    // 0x29 and collide on the bus before we can assign unique addresses.
    gpio_init(TOF_PIN_XSHUT_FL); gpio_set_dir(TOF_PIN_XSHUT_FL, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FL, 0);
    gpio_init(TOF_PIN_XSHUT_FR); gpio_set_dir(TOF_PIN_XSHUT_FR, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FR, 0);
    gpio_init(TOF_PIN_XSHUT_FM); gpio_set_dir(TOF_PIN_XSHUT_FM, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FM, 0);

    
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // CRITICAL: wait for USB CDC before any printf.
    // 3-second timeout so rover can run autonomously from battery.
    uint32_t usb_wait_start = time_us_32();
    while (!stdio_usb_connected()) {
        if ((time_us_32() - usb_wait_start) > 3000000u) break;
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(200);

    printf("\n========================================\n");
    printf("  Rover Phase 4 - Full Integration\n");
    printf("  Vision + ToF + Fusion + Odometry + Drive\n");
    printf("========================================\n\n");
    stdio_flush();

    // ── I2C init for ToF sensors ──────────────────────────────────────────────
    // GPIO function must be set BEFORE Wire0.begin() — not after
    gpio_set_function(TOF_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_PIN_SDA);
    gpio_pull_up(TOF_PIN_SCL);
    sleep_ms(10);
    Wire0.begin(100000);
    sleep_ms(50);

    // ── I2C bus scan (debug) ──────────────────────────────────────────────────
    printf("[I2C] Scanning bus...\n");
    stdio_flush();
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_timeout_us(TOF_I2C, addr, &dummy, 1, false, 10000);
        if (ret >= 0) {
            printf("[I2C] Device found at 0x%02X\n", addr);
            stdio_flush();
        }
    }
    printf("[I2C] Scan done\n");
    printf("[I2C] Ready on GP%d/GP%d\n", TOF_PIN_SDA, TOF_PIN_SCL);
    stdio_flush();

    // ── Camera init ───────────────────────────────────────────────────────────
    blink_n(2, 200);
    printf("[CAM] Starting SPI...\n"); stdio_flush();
    spi_init(CAM_SPI, 4000000);
    gpio_set_function(CAM_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(CAM_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(CAM_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(CAM_PIN_CS);
    gpio_set_dir(CAM_PIN_CS, GPIO_OUT);
    gpio_put(CAM_PIN_CS, 1);
    sleep_ms(500);

    printf("[CAM] Initializing...\n"); stdio_flush();
    myCAM.begin();
    printf("[CAM] Ready\n"); stdio_flush();
    blink_n(3, 200);

    ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t frame_buf[FRAME_BYTES];
    static uint8_t read_chunk[200];

    // ── ToF sensor init ───────────────────────────────────────────────────────
    blink_n(4, 150);
    printf("[TOF] Initializing sensors...\n");
    stdio_flush();
    if (!init_all_tof()) {
        printf("[WARN] Not all ToF sensors initialized — continuing\n");
        stdio_flush();
    }

    // ── Phase 4 subsystem init ────────────────────────────────────────────────
    printf("[P4] Initializing button...\n");       stdio_flush(); button_init();
    printf("[P4] Initializing arms...\n");          stdio_flush(); arms_init();
    
    printf("[P4] Initializing ball handler...\n"); stdio_flush(); ball_handler_init();
    
    printf("[P4] Initializing motors...\n");        stdio_flush(); motors_init();

    // ── Encoder init — must come after motors_init() ──────────────────────────
    printf("[ENC] Initializing wheel encoders...\n");
    printf("[ENC] Left=GP%d  Right=GP%d\n", ENC_PIN_LEFT, ENC_PIN_RIGHT);
    stdio_flush();
    enc.init();
    boundary.init();
    boundary.print_config();
    stdio_flush();
    blink_n(5, 100);
    
    // ══════════════════════════════════════════════════════════
    //  BALL HANDLER TEST LOOP — comment out before competition
    //  Runs infinite capture→sort→fire cycles.
    //  Unplug to stop, tune constants between runs.
    // ══════════════════════════════════════════════════════════  
   /* printf("\n[TEST] Starting ball handler test loop\n\n");
    stdio_flush();

    while (true) {
        // ── Ensure clean state before each cycle ──────────────
        flywheels_stop();
        feeder_release();
        sleep_ms(500);  // let flywheel fully coast down

        // ── Step 1: lower arms to capture position ────────────
        printf("[TEST] === NEW CYCLE ===\n"); stdio_flush();
        printf("[TEST] Lowering arms...\n"); stdio_flush();
        arms_sweep(arms_get_angle(), ARMS_ROM_MIN);
        sleep_ms(300);

        // ── Step 2: raise arms to carry position ──────────────
        printf("[TEST] Raising arms...\n"); stdio_flush();
        arms_sweep(ARMS_ROM_MIN, ARMS_ROM_MAX);
        sleep_ms(300);

        // ── Step 3: open chamber so ball drops in ─────────────
        printf("[TEST] Opening chamber...\n"); stdio_flush();
        feeder_open_chamber();
        sleep_ms(500);  // give ball time to settle into chamber

        // ── Step 4: identify ball color ───────────────────────
        printf("[TEST] Identifying color...\n"); stdio_flush();
        BallColor color = ball_handler_identify();
        printf("[TEST] Color identified: %s\n\n", color_name(color)); stdio_flush();

        // ── Step 5: fire only if ball was confirmed ───────────
        if (color == COLOR_NONE || color == COLOR_UNKNOWN || color == COLOR_ERROR) {
            printf("[TEST] No confirmed ball — closing chamber, skipping fire\n");
            stdio_flush();
            feeder_close_chamber();
        } else {
            printf("[TEST] Firing %s...\n", color_name(color)); stdio_flush();
            ball_handler_fire();
        }

        // ── Step 6: pause before next cycle ───────────────────
        printf("[TEST] Cycle done — ready for next ball\n");
        printf("[TEST] ─────────────────────────────────────\n\n");
        stdio_flush();
        sleep_ms(1000);
    }

        */

    printf("\n[READY] System ready — entering state machine\n\n");
    stdio_flush();

    // ── State machine variables ───────────────────────────────────────────────
    RoverState      state            = STATE_INIT;
    absolute_time_t state_entered_at = get_absolute_time();
    uint32_t        loop_count       = 0;

    VisionResult vision = {false, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0, 0, 0, 0};

    // ── Main control loop ─────────────────────────────────────────────────────
    while (true) {
        loop_count++;
        uint32_t loop_start = time_us_32();

        // ── Button — works from any state ─────────────────────────────────────
        if (button_pressed_edge()) {
            if (state == STATE_IDLE) {
                printf("[BTN] start -> INIT\n"); stdio_flush();
                state = STATE_INIT;
                state_entered_at = get_absolute_time();
            } else if (state == STATE_ESTOP) {
                printf("[BTN] resume -> INIT\n"); stdio_flush();
                state = STATE_INIT;
                state_entered_at = get_absolute_time();
            } else {
                printf("[BTN] e-stop\n"); stdio_flush();
                enter_estop();
                state = STATE_ESTOP;
                state_entered_at = get_absolute_time();
            }
        }

        // ── ToF read every loop (~20 Hz) ──────────────────────────────────────
        read_tof();

        // ── Camera every Nth loop (~7 Hz), active states only ─────────────────
        bool need_vision = (state == STATE_RUN    ||
                            state == STATE_CAPTURE ||
                            state == STATE_RECOVER);
        uint32_t cam_interval = (state == STATE_RUN && target_lost_count > 5) ? 1 : 3;
        if (need_vision && (loop_count % cam_interval == 0)) {
            read_camera_frame(cam, frame_buf, read_chunk);
            vision = process_frame_blob(frame_buf);
        }

        // ── Odometry update every loop during RUN ─────────────────────────────
        // Keeps pose fresh so RETURN has an accurate starting position.
        // Not needed in other states — nav functions call update_odometry()
        // themselves inside their control loops.
        if (state == STATE_RUN) {
            enc.update_odometry();
        }

        DriveCommand cmd = {0.0f, 0.0f, MODE_STOP};

        switch (state) {

            // ── IDLE ──────────────────────────────────────────────────────────
            case STATE_IDLE: {
                all_stop();
                // Slow heartbeat blink
                gpio_put(LED_PIN, ((time_us_32() / 500000) & 1) ? 1 : 0);
                break;
            }

            // ── INIT ──────────────────────────────────────────────────────────
            // 1. Arms sweep up to home
            // 2. Reset encoder — current position becomes (0, 0, 0 deg)
            // (Motor direction tests below are kept commented out for bring-up.)
            case STATE_INIT: {
                gpio_put(LED_PIN, 1);
                all_stop();

                printf("[INIT] Sweeping arms to home\n"); stdio_flush();
                arms_sweep(arms_get_angle(), ARMS_ROM_MAX);
                sleep_ms(300); 


            // ── Motor direction tests — comment out before competition ────────────────
           /*  printf("[TEST] ALL FORWARD 5s\n"); stdio_flush();
            enc.set_left_dir(1); enc.set_right_dir(1);
            motors_set(0.9f, 0.9f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);

            printf("[TEST] ALL REVERSE 5s\n"); stdio_flush();
            enc.set_left_dir(-1); enc.set_right_dir(-1);
            motors_set(-0.9f, -0.9f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);

            printf("[TEST] LEFT FORWARD only 5s\n"); stdio_flush();
            enc.set_left_dir(1); enc.set_right_dir(0);
            motors_set(0.9f, 0.0f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);

            printf("[TEST] LEFT REVERSE only 5s\n"); stdio_flush();
            enc.set_left_dir(-1); enc.set_right_dir(0);
            motors_set(-0.9f, 0.0f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);

            printf("[TEST] RIGHT FORWARD only 5s\n"); stdio_flush();
            enc.set_left_dir(0); enc.set_right_dir(1);
            motors_set(0.0f, 0.9f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);

            printf("[TEST] RIGHT REVERSE only 5s\n"); stdio_flush();
            enc.set_left_dir(0); enc.set_right_dir(-1);
            motors_set(0.0f, -0.9f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);

            printf("[TEST] LEFT FORWARD + RIGHT REVERSE 5s\n"); stdio_flush();
            enc.set_left_dir(1); enc.set_right_dir(-1);
            motors_set(0.9f, -0.9f);
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(1000);         

            printf("[TEST] Motor tests done\n"); stdio_flush();         */
            // ── End motor tests ───────────────────────────────────────────────────────
        
            // ── Dump state immediately at entry — before anything runs ────────────────
           // motors_dump_state("INIT entry");
        
            // ── Forward motor test ────────────────────────────────────────────────────
            /*printf("[INIT] Motor test — all wheels fwd 80%% for 5000ms\n"); stdio_flush();
            enc.set_left_dir(1); enc.set_right_dir(1);
            motors_set(0.8f, 0.8f);
            motors_dump_state("after motors_set(0.8, 0.8)");
            sleep_ms(5000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(300);
            printf("[INIT] Forward test done\n"); stdio_flush();
            ("after forward test");  */
        
            // ── Reverse motor test ────────────────────────────────────────────────────
           /* printf("[INIT] Motor test — all wheels REVERSE 80%% for 3s\n"); stdio_flush();
            enc.set_left_dir(-1); enc.set_right_dir(-1);
            motors_set(-0.8f, -0.8f);
            motors_dump_state("after motors_set(-0.8, -0.8)");
            sleep_ms(3000);
            motors_stop();
            enc.set_left_dir(0); enc.set_right_dir(0);
            sleep_ms(300);
            printf("[INIT] Reverse test done\n"); stdio_flush();
            //motors_dump_state("after reverse test");   */

            //motors_dump_state("after arms_sweep");
        
            // ── Ball handler init ─────────────────────────────────────────────────────
            // ball_handler_init() is called in main() setup, but re-dump here to catch
            // any side effects from arms_sweep calling into the same subsystem
            //motors_dump_state("after all subsystems");
            
        
            enc.reset_ticks();
            enc.reset_pose(0.0f, 0.0f, 0.0f);
            boundary.reset_escape_state();

            mf_init(&tof[2].filter);
            tof[2].filtered_mm    = 2000;
            capture_confirm_count = 0;

            printf("[INIT] Origin set at placement position\n");
            printf("[INIT] Role=%s  Goal position=%s 1/3\n",
                boundary.is_goalie() ? "GOALIE" : "KICKER",
                boundary.is_left_corner() ? "LEFT" : "RIGHT");
            printf("[INIT] MM_PER_TICK=%.4f\n", MM_PER_TICK);
            stdio_flush();

            printf("[STATE] INIT -> RUN\n"); stdio_flush();
            state             = STATE_RUN;
            state_entered_at  = get_absolute_time();
            target_lost_count = 0;
            break;
        }

            // ── RUN ───────────────────────────────────────────────────────────
            case STATE_RUN: {
            CollisionResult collision = get_collision();
            cmd = compute_drive(vision, collision);

            // Capture trigger:
            // FM ToF must confirm ball is physically in arm sweep range.
            // Vision (target_size >= 30) is used while far, but dropped at close
            // range because the ball clips the bottom of the 128x128 frame and
            // target_size becomes unreliable. ToF alone is sufficient once we are
            // solidly in-zone — the ball is right there.
            bool fm_solidly_in_zone = (tof[2].filtered_mm >= FM_CAPTURE_MIN_MM &&
                                        tof[2].filtered_mm <= FM_CAPTURE_MAX_MM);
            bool vision_tracking    = (cmd.mode == MODE_TRACK ||
                                        cmd.mode == MODE_APPROACH);

            // Vision confirms while far. ToF alone confirms at close range.
            bool capture_condition  = collision.ball_in_capture_zone &&
                                    (vision.target_size >= 30 || fm_solidly_in_zone);

            if (vision_tracking && capture_condition) {
                capture_confirm_count++;
                printf("[CAP] confirm=%d/5 FM=%u sz=%u mode=%s vision=%s\n",
                    capture_confirm_count,
                    collision.fm_mm,
                    vision.target_size,
                    mode_names[cmd.mode],
                    vision.target_found ? "YES" : "NO");
                stdio_flush();

                if (capture_confirm_count >= 5) {
                    capture_confirm_count = 0;
                    printf("[STATE] RUN -> CAPTURE (FM=%umm FL=%umm FR=%umm sz=%u)\n",
                        collision.fm_mm, collision.fl_mm, collision.fr_mm,
                        vision.target_size);
                    stdio_flush();
                    motors_stop();
                    enc.set_left_dir(0);
                    enc.set_right_dir(0);
                    sleep_ms(150);
                    state            = STATE_CAPTURE;
                    state_entered_at = get_absolute_time();
                    break;
                }
            } else {
                if (capture_confirm_count > 0) {
                    printf("[CAP] reset — fm=%u in_zone=%s vision_tracking=%s capture_cond=%s\n",
                        tof[2].filtered_mm,
                        fm_solidly_in_zone   ? "YES" : "NO",
                        vision_tracking      ? "YES" : "NO",
                        capture_condition    ? "YES" : "NO");
                    stdio_flush();
                }
                capture_confirm_count = 0;
            }

            BoundaryAction ba = boundary.check(enc.x(), enc.y());
            if (ba == BoundaryAction::BRAKE) {
                BoundaryCmd bc = boundary.correction_command(enc.x(), enc.y(), enc.theta());
                enc.set_left_dir (bc.left  > 0.01f ? 1 : bc.left  < -0.01f ? -1 : 0);
                enc.set_right_dir(bc.right > 0.01f ? 1 : bc.right < -0.01f ? -1 : 0);
                motors_set(bc.left, bc.right);
                target_lost_count    = 0;
                search_forward_until = 0;
                search_cycle_count   = 0;
            } else if (ba == BoundaryAction::WARN) {
                BoundaryCmd bc = boundary.correction_command(enc.x(), enc.y(), enc.theta());
                enc.set_left_dir (bc.left  > 0.01f ? 1 : bc.left  < -0.01f ? -1 : 0);
                enc.set_right_dir(bc.right > 0.01f ? 1 : bc.right < -0.01f ? -1 : 0);
                motors_set(bc.left, bc.right);
            } else {
                apply_drive_command(cmd);
            }
            break;
        }  // ← closes STATE_RUN

            // ── CAPTURE ───────────────────────────────────────────────────────
            // Ball confirmed in position by FM ToF. Stop, scoop, lift.
            case STATE_CAPTURE: {
                motors_stop();
                enc.set_left_dir(0);
                enc.set_right_dir(0);

                printf("[STATE] CAPTURE - lowering arm\n");
                stdio_flush();
                arms_sweep(ARMS_ROM_MAX, ARMS_ROM_MIN);  // lower onto ball
                sleep_ms(200);

                printf("[STATE] CAPTURE - raising arm\n");
                stdio_flush();
                arms_sweep(ARMS_ROM_MIN, ARMS_ROM_MAX);  // raise into funnel
                

                // Open chamber so ball falls in from funnel
                feeder_open_chamber();

                // Flush FM filter — arms sweeping during capture leave close readings
                // that would immediately re-trigger capture when returning to RUN
                mf_init(&tof[2].filter);
                tof[2].filtered_mm = 2000;
                capture_confirm_count  = 0;

                printf("[STATE] CAPTURE -> SORT\n");
                stdio_flush();
                state = STATE_SORT;
                state_entered_at = get_absolute_time();
                break;
            }

            // ── SORT ──────────────────────────────────────────────────────────
            // Identify the ball color and store it. Do NOT fire yet.
            // The rover returns to origin first (STATE_RETURN), then fires
            // from the same position every time (STATE_FIRE).
            case STATE_SORT: {
                motors_stop();
                enc.set_left_dir(0);
                enc.set_right_dir(0);
 
                printf("[STATE] SORT - identifying ball color (no firing yet)\n");
                stdio_flush();
 
                BallColor identified = ball_handler_identify();
                printf("[STATE] SORT identified: %s\n", color_name(identified));
                stdio_flush();

                if (identified == COLOR_NONE || identified == COLOR_UNKNOWN || identified == COLOR_ERROR) {
                    // No ball confirmed — close chamber and go back to searching
                    printf("[STATE] SORT no ball — closing chamber -> RUN\n");
                    stdio_flush();
                    feeder_close_chamber();
                    state = STATE_RUN;
                } else {
                    printf("[STATE] SORT confirmed %s -> RETURN\n", color_name(identified));
                    stdio_flush();
                    state = STATE_RETURN;
                }
                state_entered_at = get_absolute_time();
                target_lost_count = 0;
                break;
            }

            // ── RETURN ────────────────────────────────────────────────────────
            case STATE_RETURN: {
                nav_return_to_origin();
                printf("[STATE] RETURN  (encoders on) -> FIRE\n");
                stdio_flush();
                state = STATE_FIRE;
                state_entered_at = get_absolute_time();
                break;
            }

            // ── FIRE ──────────────────────────────────────────────────────────
            // Rover is now at origin, facing 0 deg (toward opponent goal).
            // Fire the feeder based on the color stored during SORT.
            case STATE_FIRE: {
                    motors_stop();

                    BallColor c = ball_handler_get_last_color();
                    printf("[STATE] FIRE — color: %s\n", color_name(c));
                    stdio_flush();

                    if (c == COLOR_RED) {
                        // Already facing opponent at theta=0 — shoot across field
                        printf("[STATE] FIRE RED — shooting toward opponent goal\n");
                        stdio_flush();
                        ball_handler_fire();
                    } else {
                        // GREEN/BLUE — eject backward, ball rolls 381mm into own goal
                        printf("[STATE] FIRE %s — ejecting toward own goal\n", color_name(c));
                        stdio_flush();
                        ball_handler_fire();   // ball_handler_fire() handles eject for non-RED
                    }

                    printf("[STATE] FIRE -> RECOVER\n"); stdio_flush();
                    state            = STATE_RECOVER;
                    state_entered_at = get_absolute_time();
                    break;
                }

            // ── RECOVER ───────────────────────────────────────────────────────
            case STATE_RECOVER: {
                cmd.linear_speed = RECOVER_SPEED;
                cmd.turn_rate    = 0.0f;
                cmd.mode         = MODE_REVERSE;
                apply_drive_command(cmd);

                int64_t elapsed_ms =
                    absolute_time_diff_us(state_entered_at, get_absolute_time()) / 1000;
                if (elapsed_ms >= (int64_t)RECOVER_DRIVE_MS) {
                    motors_stop();
                    printf("[STATE] RECOVER -> RUN\n");
                    stdio_flush();
                    state             = STATE_RUN;
                    state_entered_at  = get_absolute_time();
                    target_lost_count = 0;
                }
                break;
            }

            // ── ESTOP ─────────────────────────────────────────────────────────
            case STATE_ESTOP: {
                all_stop();
                gpio_put(LED_PIN, ((time_us_32() / 100000) & 1) ? 1 : 0);
                break;
            }
        }

        // ── Periodic debug print (every 5 loops) ──────────────────────────────
        // Skipped during RETURN — nav_return_to_origin() prints its own output.
        // Also skipped during IDLE and ESTOP (motors stopped, nothing changing).
        if (loop_count % 5 == 0 &&
            state != STATE_IDLE   &&
            state != STATE_ESTOP  &&
            state != STATE_RETURN) {

            uint32_t loop_us = time_us_32() - loop_start;
            BoundaryAction ba_dbg = boundary.check(enc.x(), enc.y());
            printf("[%s/%s] spd=%+.2f trn=%+.2f | pos=(%.0f,%.0f) theta=%.1fdeg | bnd=%s | ",
                state_name(state), mode_names[cmd.mode],
                cmd.linear_speed, cmd.turn_rate,
                enc.x(), enc.y(),
                enc.theta() * 180.0f / (float)M_PI,
                ba_dbg == BoundaryAction::BRAKE ? "BRAKE" :
                ba_dbg == BoundaryAction::WARN  ? "WARN"  : "SAFE");

            if (vision.target_found)
                printf("ball:x=%+.2f sz=%u | ", vision.target_x, vision.target_size);
            else
                printf("ball:--- | ");

            printf("FL:%4u[%s] FR:%4u[%s] FM:%4u[%s] | %lums\n",
                   tof[0].filtered_mm, zone_names[tof[0].zone],
                   tof[1].filtered_mm, zone_names[tof[1].zone],
                   tof[2].filtered_mm, zone_names[tof[2].zone],
                   (unsigned long)(loop_us / 1000));
            stdio_flush();
        }

        sleep_ms(30);
    }
}