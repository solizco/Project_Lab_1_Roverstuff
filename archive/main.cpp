/*
 * Rover Phase 4 — Full Integration
 * Board:  Pico 2 (RP2350)
 * Camera: Arducam Mega 3MP (SPI) — 128x128 RGB565 red detection
 * ToF:    2x VL53L0X (FL, FR) + 1x VL53L1X (FM) on shared I2C0
 * Ball:   MG995 arms + TCS3200 color sensor + feeder servo + flywheel
 * Drive:  Custom H-bridge (Guy) — placeholder pins
 *
 * This is Phase 3 (vision + ToF + fusion) PLUS:
 *   - Top-level state machine (IDLE, INIT, RUN, CAPTURE, SORT, RECOVER, ESTOP)
 *   - Start/E-stop button (BTN_PIN)
 *   - Arms sweep up at INIT, lower/raise during CAPTURE
 *   - Camera steers rover to center ball, FM ToF triggers capture
 *     when ball is 45-100mm from sensor (physically in arm position)
 *   - FL/FR clear during FM trigger rules out another rover
 *   - Drive motor output via motors_set()
 *
 * Concurrency: single core, cooperative loop. The SORT state is the only
 * one that blocks (~5 s); drive motors are commanded to STOP first.
 *
 * State flow:
 *   IDLE  --button-->  INIT (arms up) --> RUN <--> (TRACK/SEARCH/AVOID)
 *                                          |
 *                                          v
 *                                       CAPTURE (FM 45-100mm + FL/FR clear)
 *                                          |
 *                                          v
 *                                        SORT (blocking)
 *                                          |
 *                                          v
 *                                       RECOVER --> RUN
 *
 *   any state --button--> ESTOP --button--> INIT (resume)
 *
 * Sensor wiring on shared I2C0 (GP4 SDA, GP5 SCL):
 *   FL = VL53L0X @ 0x30  (XSHUT GP8, purple)
 *   FR = VL53L0X @ 0x31  (XSHUT GP6, blue)
 *   FM = VL53L1X @ 0x32  (XSHUT GP7, gray)
 *
 * Library change (Phase 4 rev):
 *   Previously used C-style VL53L1X_api / vl53l0x_min free-function APIs.
 *   Now uses Pololu Arduino-style VL53L0X and VL53L1X C++ classes via a
 *   thin TwoWire shim (pico_arduino_wire_shim.h) that wraps pico-sdk i2c.
 *   Key API mapping:
 *     OLD: vl53l0x_init() / vl53l0x_set_address() / vl53l0x_read_mm()
 *     NEW: VL53L0X::init() / VL53L0X::setAddress() / VL53L0X::readRangeContinuousMillimeters()
 *
 *     OLD: VL53L1X_I2C_Init() / VL53L1X_SensorInit() / VL53L1X_GetDistance()
 *     NEW: VL53L1X::init() / VL53L1X::setDistanceMode() / VL53L1X::read(false)
 */

#include <cstdio>
#include <cstring>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "Arducam_Mega.h"
#include "ArducamCamera.h"

// Pololu-style C++ sensor classes (require a TwoWire / Wire shim for Pico SDK).
// The shim must implement: beginTransmission, write, endTransmission,
// requestFrom, read — wrapping i2c_write_blocking / i2c_read_blocking.
#include "Wire.h"   // project-level TwoWire wrapper
#include "VL53L0X.h"
#include "VL53L1X.h"

#include "config_pins.h"
#include "button.h"
#include "collector_arms.h"
#include "ball_handler.h"
#include "rover_motors.h"
#include "flywheel_motors.h"
#include "rack_pinion.h"

// ============================================================
//  CAMERA VISION (unchanged from Phase 3)
// ============================================================

#define FRAME_W 128
#define FRAME_H 128
#define FRAME_PIXELS (FRAME_W * FRAME_H)
#define FRAME_BYTES  (FRAME_PIXELS * 2)

static Arducam_Mega myCAM(CAM_PIN_CS);

static inline uint8_t rgb565_red(uint8_t hi, uint8_t lo)   { return (hi & 0xF8); }
static inline uint8_t rgb565_green(uint8_t hi, uint8_t lo) { return ((hi & 0x07) << 5) | ((lo & 0xE0) >> 3); }
static inline uint8_t rgb565_blue(uint8_t hi, uint8_t lo)  { return (lo & 0x1F) << 3; }

static const uint8_t RED_MIN_R = 140;
static const uint8_t RED_MAX_G = 80;
static const uint8_t RED_MAX_B = 80;

struct VisionResult {
    bool     target_found;
    float    target_x;      // -1.0 (left) to +1.0 (right)
    float    target_y;      // -1.0 (top) to +1.0 (bottom)
    uint16_t target_size;   // red pixel count
    float    confidence;
};

static VisionResult process_frame_rgb565(const uint8_t* frame_data) {
    VisionResult result = {false, 0.0f, 0.0f, 0, 0.0f};

    uint32_t red_count = 0;
    int32_t  red_x_sum = 0;
    int32_t  red_y_sum = 0;

    for (int y = 0; y < FRAME_H; y++) {
        for (int x = 0; x < FRAME_W; x++) {
            int idx = (y * FRAME_W + x) * 2;
            uint8_t hi = frame_data[idx];
            uint8_t lo = frame_data[idx + 1];

            uint8_t r = rgb565_red(hi, lo);
            uint8_t g = rgb565_green(hi, lo);
            uint8_t b = rgb565_blue(hi, lo);

            if (r >= RED_MIN_R && g <= RED_MAX_G && b <= RED_MAX_B) {
                red_count++;
                red_x_sum += x;
                red_y_sum += y;
            }
        }
    }

    if (red_count > 8) {
        result.target_found = true;
        float avg_x = (float)red_x_sum / (float)red_count;
        float avg_y = (float)red_y_sum / (float)red_count;
        result.target_x = (avg_x - (FRAME_W / 2.0f)) / (FRAME_W / 2.0f);
        result.target_y = (avg_y - (FRAME_H / 2.0f)) / (FRAME_H / 2.0f);
        result.target_size = (uint16_t)red_count;
        result.confidence = (float)red_count / (float)FRAME_PIXELS;
    }

    return result;
}

static void read_camera_frame(ArducamCamera* cam, uint8_t* frame_buf, uint8_t* read_chunk) {
    myCAM.takePicture((CAM_IMAGE_MODE)CAM_IMAGE_MODE_128X128,
                      (CAM_IMAGE_PIX_FMT)CAM_IMAGE_PIX_FMT_RGB565);
    uint32_t total = myCAM.getTotalLength();

    uint32_t bytesRead = 0;
    while (bytesRead < total && bytesRead < FRAME_BYTES) {
        uint32_t remaining = total - bytesRead;
        uint8_t chunk = (remaining > 200) ? 200 : (uint8_t)remaining;
        readBuff(cam, read_chunk, chunk);
        uint32_t copyLen = chunk;
        if (bytesRead + copyLen > FRAME_BYTES) copyLen = FRAME_BYTES - bytesRead;
        memcpy(&frame_buf[bytesRead], read_chunk, copyLen);
        bytesRead += chunk;
    }
    while (bytesRead < total) {
        uint32_t remaining = total - bytesRead;
        uint8_t chunk = (remaining > 200) ? 200 : (uint8_t)remaining;
        readBuff(cam, read_chunk, chunk);
        bytesRead += chunk;
    }
}

// ============================================================
//  TOF COLLISION AVOIDANCE — mixed L0X (FL/FR) + L1X (FM)
//
//  The Pololu VL53L0X / VL53L1X classes communicate through a
//  TwoWire object.  We create one shim instance for the shared
//  I2C bus, assign it to each sensor object with setBus(), then
//  use the class API instead of the old free functions:
//
//    sensor.setBus(&Wire0)       → replaces i2c port registration
//    sensor.setAddress(addr)     → replaces *_set_address()
//    sensor.init()               → replaces *_init() / SensorInit()
//    sensor.startContinuous(ms)  → replaces StartRanging()
//    sensor.read(false)          / sensor.readRangeContinuousMillimeters()
//                                → replaces CheckForDataReady + GetDistance
// ============================================================

#define NUM_SENSORS 3

// Default I2C address both sensor families boot at
#define VL53_DEFAULT_ADDR 0x29

// Final reassigned addresses (one per slot, must be unique and != 0x29)
static const uint8_t SENSOR_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32};

// XSHUT GPIO pins — held LOW to keep each sensor in hardware reset
static const uint8_t XSHUT_PINS[NUM_SENSORS] = {
    TOF_PIN_XSHUT_FL,   // FL  VL53L0X
    TOF_PIN_XSHUT_FR,   // FR  VL53L0X
    TOF_PIN_XSHUT_FM    // FM  VL53L1X
};

static const char* SENSOR_NAMES[NUM_SENSORS] = {"FL", "FR", "FM"};

#define STOP_DISTANCE_MM  150
#define SLOW_DISTANCE_MM  250

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
    std::sort(sorted, sorted + f->count);
    return sorted[f->count / 2];
}

typedef struct {
    uint16_t     filtered_mm;
    DistanceZone zone;
    bool         initialized;
    MedianFilter filter;
} ToFSensor;

static ToFSensor tof[NUM_SENSORS];

// One TwoWire shim for the shared I2C bus (constructed with pico-sdk i2c_inst_t*)
static TwoWire Wire0(TOF_I2C);

// Sensor objects — two L0X (FL, FR) and one L1X (FM)
static VL53L0X sensorFL;
static VL53L0X sensorFR;
static VL53L1X sensorFM;

static DistanceZone classify(uint16_t mm) {
    if (mm == 0) return ZONE_ERROR;
    if (mm < STOP_DISTANCE_MM) return ZONE_STOP;
    if (mm < SLOW_DISTANCE_MM) return ZONE_SLOW;
    return ZONE_CLEAR;
}

// CHANGE 2: CollisionResult — removed danger_rear/rr_mm/rr_zone,
// added ball_in_capture_zone/fm_mm/fm_zone and FM threshold defines
#define FM_CAPTURE_MIN_MM  45   // ball must be at least this close
#define FM_CAPTURE_MAX_MM  100  // ball must be no further than this
#define SIDE_CLEAR_MM      150  // FL/FR must exceed this to confirm ball not rover

struct CollisionResult {
    bool danger_left;
    bool danger_right;
    bool front_blocked;
    bool ball_in_capture_zone;  // FM in range AND FL/FR clear = ball confirmed
    DistanceZone fl_zone, fr_zone, fm_zone;
    uint16_t fl_mm, fr_mm, fm_mm;
};

// ── Init a single L0X sensor (FL or FR) ────────────────────────────────────
// All sensors remain in XSHUT reset except the one being brought up.
// After boot the sensor answers at VL53_DEFAULT_ADDR (0x29); we reassign it
// before releasing the next sensor so there is never an address collision.
static bool init_l0x_sensor(int idx, VL53L0X& sensor) {
    tof[idx].initialized = false;
    mf_init(&tof[idx].filter);

    // Release ONLY this sensor from reset
    gpio_put(XSHUT_PINS[idx], 1);
    sleep_ms(50);

    // DEBUG: scan bus right after releasing this sensor
    printf("[TOF] %s: scanning bus after XSHUT release...\n", SENSOR_NAMES[idx]);
    stdio_flush();
    bool found_anything = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        uint8_t dummy;
        if (i2c_read_timeout_us(TOF_I2C, a, &dummy, 1, false, 10000) >= 0) {
            printf("[TOF] %s: found device at 0x%02X\n", SENSOR_NAMES[idx], a);
            stdio_flush();
            found_anything = true;
        }
    }
    if (!found_anything) {
        printf("[TOF] %s: nothing on bus after XSHUT release\n", SENSOR_NAMES[idx]);
        stdio_flush();
    }

    // Point the library object at our shared TwoWire shim; it addresses the
    // sensor at the default boot address until setAddress() is called.
    sensor.setBus(&Wire0);
    sensor.setTimeout(500);

    // init() checks model-ID register and performs full ST-recommended init
    if (!sensor.init()) {
        printf("[TOF] %s: L0X init() failed, skipping\n", SENSOR_NAMES[idx]);
        stdio_flush();
        gpio_put(XSHUT_PINS[idx], 0);
        return false;
    }

    // Reassign I2C address so the next sensor can boot at 0x29 safely
    sensor.setAddress(SENSOR_ADDRS[idx]);
    sleep_ms(10);

    // Verify the sensor ACKs at its new address via the low-level SDK
    uint8_t dummy;
    if (i2c_read_timeout_us(TOF_I2C, SENSOR_ADDRS[idx], &dummy, 1, false, 10000) < 0) {
        printf("[TOF] %s: L0X did not respond after addr change\n", SENSOR_NAMES[idx]);
        stdio_flush();
        gpio_put(XSHUT_PINS[idx], 0);
        return false;
    }

    // Start back-to-back continuous ranging (0 ms = fastest rate)
    sensor.startContinuous(0);

    tof[idx].initialized = true;
    printf("[TOF] %s: L0X OK at 0x%02X\n", SENSOR_NAMES[idx], SENSOR_ADDRS[idx]);
    stdio_flush();
    return true;
}

// ── Init the L1X sensor (FM) ────────────────────────────────────────────────
static bool init_l1x_sensor(int idx, VL53L1X& sensor) {
    tof[idx].initialized = false;
    mf_init(&tof[idx].filter);

    // Release ONLY this sensor from reset
    gpio_put(XSHUT_PINS[idx], 1);
    sleep_ms(50);

    // DEBUG: scan bus right after releasing this sensor
    printf("[TOF] %s: scanning bus after XSHUT release...\n", SENSOR_NAMES[idx]);
    stdio_flush();
    bool found_anything = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        uint8_t dummy;
        if (i2c_read_timeout_us(TOF_I2C, a, &dummy, 1, false, 10000) >= 0) {
            printf("[TOF] %s: found device at 0x%02X\n", SENSOR_NAMES[idx], a);
            stdio_flush();
            found_anything = true;
        }
    }
    if (!found_anything) {
        printf("[TOF] %s: nothing on bus after XSHUT release\n", SENSOR_NAMES[idx]);
        stdio_flush();
    }

    // Point the library object at our shared TwoWire shim
    sensor.setBus(&Wire0);
    sensor.setTimeout(500);

    // init() checks model-ID (0xEACC), soft-resets, polls boot-complete,
    // and applies ST-recommended static / dynamic config
    if (!sensor.init()) {
        printf("[TOF] %s: L1X init() failed, skipping\n", SENSOR_NAMES[idx]);
        stdio_flush();
        gpio_put(XSHUT_PINS[idx], 0);
        return false;
    }

    // Short distance mode is best for the 45-100mm ball capture range
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(50000); // 50 ms

    // Reassign I2C address so the slot is permanently distinguishable
    printf("[TOF] %s: L1X assigning address 0x%02X\n", SENSOR_NAMES[idx], SENSOR_ADDRS[idx]);
    stdio_flush();
    sensor.setAddress(SENSOR_ADDRS[idx]);
    sleep_ms(10);

    // Start continuous ranging with 55 ms inter-measurement period
    sensor.startContinuous(55);

    tof[idx].initialized = true;
    printf("[TOF] %s: L1X OK at 0x%02X\n", SENSOR_NAMES[idx], SENSOR_ADDRS[idx]);
    stdio_flush();
    return true;
}

static bool init_all_tof() {
    int ok = 0;

    gpio_put(TOF_PIN_XSHUT_FL, 0);
    gpio_put(TOF_PIN_XSHUT_FR, 0);
    gpio_put(TOF_PIN_XSHUT_FM, 0);
    sleep_ms(20);

    // ── FL: VL53L0X ──────────────────────────────────────────
    gpio_put(TOF_PIN_XSHUT_FL, 1);
    sleep_ms(100);
    sensorFL.setBus(&Wire0);
    sensorFL.setTimeout(500);
    if (!sensorFL.init()) {
        printf("[TOF] FL: init() failed\n"); stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FL, 0);
    } else {
        sensorFL.setAddress(SENSOR_ADDRS[0]);
        sleep_ms(20);
        // startContinuous moved to end
        tof[0].initialized = true;
        mf_init(&tof[0].filter);
        printf("[TOF] FL: OK at 0x%02X\n", SENSOR_ADDRS[0]); stdio_flush();
        ok++;
    }

    // ── FR: VL53L0X ──────────────────────────────────────────
    gpio_put(TOF_PIN_XSHUT_FR, 1);
    sleep_ms(100);
    sensorFR.setBus(&Wire0);
    sensorFR.setTimeout(500);
    if (!sensorFR.init()) {
        printf("[TOF] FR: init() failed\n"); stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FR, 0);
    } else {
        sensorFR.setAddress(SENSOR_ADDRS[1]);
        sleep_ms(20);
        // startContinuous moved to end
        tof[1].initialized = true;
        mf_init(&tof[1].filter);
        printf("[TOF] FR: OK at 0x%02X\n", SENSOR_ADDRS[1]); stdio_flush();
        ok++;
    }

    // ── FM: VL53L1X ──────────────────────────────────────────
    gpio_put(TOF_PIN_XSHUT_FM, 1);
    sleep_ms(100);
    sensorFM.setBus(&Wire0);
    sensorFM.setTimeout(500);
    if (!sensorFM.init()) {
        printf("[TOF] FM: init() failed\n"); stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FM, 0);
    } else {
        sensorFM.setAddress(SENSOR_ADDRS[2]);
        sleep_ms(20);
        sensorFM.setDistanceMode(VL53L1X::Short);
        sensorFM.setMeasurementTimingBudget(50000);
        // startContinuous moved to end
        tof[2].initialized = true;
        mf_init(&tof[2].filter);
        printf("[TOF] FM: OK at 0x%02X\n", SENSOR_ADDRS[2]); stdio_flush();
        ok++;
    }

    // ── Start continuous AFTER all address assignments are done ──
    printf("[TOF] Starting continuous ranging...\n"); stdio_flush();
    if (tof[0].initialized) sensorFL.startContinuous(0);
    if (tof[1].initialized) sensorFR.startContinuous(0);
    if (tof[2].initialized) sensorFM.startContinuous(55);

    printf("[TOF] %d/3 sensors ready\n", ok); stdio_flush();
    return (ok >= 2);
}

// ── Read all sensors, dispatching per type ───────────────────────────────────
// L0X: readRangeContinuousMillimeters() polls the interrupt flag internally
//      and blocks until data is ready — acceptable at ~20 Hz loop rate.
// L1X: read(false) is non-blocking; we only update the median filter when
//      dataReady() confirms a fresh measurement has arrived.
static void read_tof() {
    // ── FL (VL53L0X) ──
    if (tof[0].initialized) {
        uint16_t d = sensorFL.readRangeContinuousMillimeters();
        if (!sensorFL.timeoutOccurred() && d > 0 && d < 8000) {
            mf_add(&tof[0].filter, d);
            tof[0].filtered_mm = mf_get(&tof[0].filter);
        }
        tof[0].zone = classify(tof[0].filtered_mm);
    } else {
        tof[0].zone = ZONE_ERROR;
    }

    // ── FR (VL53L0X) ──
    if (tof[1].initialized) {
        uint16_t d = sensorFR.readRangeContinuousMillimeters();
        if (!sensorFR.timeoutOccurred() && d > 0 && d < 8000) {
            mf_add(&tof[1].filter, d);
            tof[1].filtered_mm = mf_get(&tof[1].filter);
        }
        tof[1].zone = classify(tof[1].filtered_mm);
    } else {
        tof[1].zone = ZONE_ERROR;
    }

    // ── FM (VL53L1X) — non-blocking poll ──
    // dataReady() reads the GPIO interrupt line; read(false) fetches the result
    // and clears the interrupt without blocking if called only when ready.
    if (tof[2].initialized) {
        if (sensorFM.dataReady()) {
            uint16_t d = sensorFM.read(false); // non-blocking, clears interrupt
            if (!sensorFM.timeoutOccurred() && d > 0 && d < 8000) {
                mf_add(&tof[2].filter, d);
                tof[2].filtered_mm = mf_get(&tof[2].filter);
            }
        }
        tof[2].zone = classify(tof[2].filtered_mm);
    } else {
        tof[2].zone = ZONE_ERROR;
    }
}

// CHANGE 3: get_collision() — replaced rr with fm, added ball_in_capture_zone logic
static CollisionResult get_collision() {
    CollisionResult c;
    c.fl_mm   = tof[0].filtered_mm;
    c.fr_mm   = tof[1].filtered_mm;
    c.fm_mm   = tof[2].filtered_mm;
    c.fl_zone = tof[0].zone;
    c.fr_zone = tof[1].zone;
    c.fm_zone = tof[2].zone;
    c.danger_left  = (tof[0].zone == ZONE_STOP);
    c.danger_right = (tof[1].zone == ZONE_STOP);
    uint16_t front_min = std::min(c.fl_mm, c.fr_mm);
    c.front_blocked = (front_min > 0 && front_min < STOP_DISTANCE_MM);

    // Ball confirmed when FM is in capture range AND FL/FR are both clear.
    // If FL/FR were also close, it would indicate a rover-sized object, not a ball.
    bool fm_in_range = (c.fm_mm >= FM_CAPTURE_MIN_MM && c.fm_mm <= FM_CAPTURE_MAX_MM);
    bool sides_clear = (c.fl_mm == 0 || c.fl_mm > SIDE_CLEAR_MM) &&
                       (c.fr_mm == 0 || c.fr_mm > SIDE_CLEAR_MM);
    c.ball_in_capture_zone = fm_in_range && sides_clear;
    return c;
}

// ============================================================
//  DRIVE COMMAND & FUSION
// ============================================================

typedef enum {
    MODE_SEARCH,
    MODE_TRACK,
    MODE_AVOID,
    MODE_REVERSE,
    MODE_STOP
} DriveMode;

static const char* mode_names[] = {"SEARCH", "TRACK ", "AVOID ", "REVRSE", "STOP  "};

struct DriveCommand {
    float     linear_speed;
    float     turn_rate;
    DriveMode mode;
};

static const float SPEED_FULL    = 0.8f;
static const float SPEED_SLOW    = 0.5f;
static const float SPEED_REVERSE = -0.5f;
static const float TURN_TRACK    = 0.5f;
static const float TURN_AVOID    = 0.3f;
static const float TURN_SEARCH   = 0.8f;

static uint32_t target_lost_count = 0;
static const uint32_t SEARCH_TIMEOUT = 500;

// CHANGE 4: compute_drive() — removed danger_rear boxed-in STOP case.
// Front blocked always reverses since there is no rear sensor.
static DriveCommand compute_drive(const VisionResult& vision, const CollisionResult& collision) {
    DriveCommand cmd = {0.0f, 0.0f, MODE_STOP};

    if (collision.front_blocked) {
        cmd.mode = MODE_REVERSE;
        cmd.linear_speed = SPEED_REVERSE;
        if (collision.fl_mm < collision.fr_mm) {
            cmd.turn_rate = TURN_AVOID;
        } else {
            cmd.turn_rate = -TURN_AVOID;
        }
        target_lost_count = 0;
        return cmd;
    }

    if (collision.danger_left && !collision.danger_right) {
        cmd.mode = MODE_AVOID;
        cmd.linear_speed = SPEED_SLOW;
        cmd.turn_rate = TURN_AVOID;
        return cmd;
    }
    if (collision.danger_right && !collision.danger_left) {
        cmd.mode = MODE_AVOID;
        cmd.linear_speed = SPEED_SLOW;
        cmd.turn_rate = -TURN_AVOID;
        return cmd;
    }

    if (vision.target_found) {
        target_lost_count = 0;
        cmd.mode = MODE_TRACK;
        cmd.turn_rate = vision.target_x * TURN_TRACK;
        if (vision.target_size > 500) {
            cmd.linear_speed = SPEED_SLOW;
        } else {
            cmd.linear_speed = SPEED_FULL;
        }
        if (tof[0].zone == ZONE_SLOW || tof[1].zone == ZONE_SLOW) {
            cmd.linear_speed = SPEED_SLOW;
        }
        return cmd;
    }

    target_lost_count++;
    cmd.mode = MODE_SEARCH;
    cmd.linear_speed = 0.0f;

    if (target_lost_count > SEARCH_TIMEOUT) {
        cmd.turn_rate = ((target_lost_count / 50) % 2 == 0) ? TURN_SEARCH : -TURN_SEARCH;
        cmd.linear_speed = SPEED_SLOW;
    } else {
        cmd.turn_rate = TURN_SEARCH;
    }

    return cmd;
}

// ============================================================
//  PHASE 4: TOP-LEVEL STATE MACHINE
// ============================================================

enum RoverState {
    STATE_IDLE = 0,
    STATE_INIT,
    STATE_RUN,
    STATE_CAPTURE,
    STATE_SORT,
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
        case STATE_RECOVER: return "RECOVER";
        case STATE_ESTOP:   return "ESTOP";
    }
    return "?";
}

// CHANGE 5: Removed CAPTURE_PIXEL_THRESHOLD, CAPTURE_DRIVE_MS, CAPTURE_SPEED —
// FM ToF is now the capture trigger and CAPTURE no longer drives forward.
static const uint32_t RECOVER_DRIVE_MS = 500;
static const float    RECOVER_SPEED    = -0.40f;

static void apply_drive_command(const DriveCommand &cmd) {
    float left  = cmd.linear_speed + cmd.turn_rate;
    float right = cmd.linear_speed - cmd.turn_rate;
    if (left  >  1.0f) left  =  1.0f;
    if (left  < -1.0f) left  = -1.0f;
    if (right >  1.0f) right =  1.0f;
    if (right < -1.0f) right = -1.0f;
    printf("[DRIVE] L=%.2f R=%.2f\n", left, right);
    stdio_flush();
    motors_set(left, right);
}

static void all_stop() {
    motors_stop();
    flywheels_stop();
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

int main()
{
    // Pull all motor pins LOW immediately to prevent float-driven spin at boot
    gpio_init(MOTOR_L_PIN_IN3); gpio_set_dir(MOTOR_L_PIN_IN3, GPIO_OUT); gpio_put(MOTOR_L_PIN_IN3, 0);
    gpio_init(MOTOR_L_PIN_IN4); gpio_set_dir(MOTOR_L_PIN_IN4, GPIO_OUT); gpio_put(MOTOR_L_PIN_IN4, 0);
    gpio_init(MOTOR_R_PIN_IN1); gpio_set_dir(MOTOR_R_PIN_IN1, GPIO_OUT); gpio_put(MOTOR_R_PIN_IN1, 0);
    gpio_init(MOTOR_R_PIN_IN2); gpio_set_dir(MOTOR_R_PIN_IN2, GPIO_OUT); gpio_put(MOTOR_R_PIN_IN2, 0);

    // Pull all ToF XSHUT pins LOW immediately. Both VL53L0X and VL53L1X have
    // an internal pull-up on XSHUT — if left floating during the USB-wait
    // loop below, all sensors boot together at 0x29 and collide on the bus.
    // This must happen BEFORE stdio_init_all / USB wait.
    gpio_init(TOF_PIN_XSHUT_FL); gpio_set_dir(TOF_PIN_XSHUT_FL, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FL, 0);
    gpio_init(TOF_PIN_XSHUT_FR); gpio_set_dir(TOF_PIN_XSHUT_FR, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FR, 0);
    gpio_init(TOF_PIN_XSHUT_FM); gpio_set_dir(TOF_PIN_XSHUT_FM, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FM, 0);

    // ... rest of main
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // CRITICAL: wait for USB CDC before any printf
    // Wait up to 3 seconds for USB, then continue anyway
    uint32_t usb_wait_start = time_us_32();
    while (!stdio_usb_connected()) {
        if ((time_us_32() - usb_wait_start) > 3000000) break; // 3s timeout
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(200);

    printf("\n========================================\n");
    printf("  Rover Phase 4 - Full Integration\n");
    printf("  Vision + ToF (L0X+L1X) + Fusion + Ball + Drive\n");
    printf("========================================\n\n");
    stdio_flush();

    // ── Init I2C for ToF sensors ──
    // Set pins FIRST, then begin
    gpio_set_function(TOF_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_PIN_SDA);
    gpio_pull_up(TOF_PIN_SCL);
    sleep_ms(10);
    Wire0.begin(100000);  // NOW initialize hardware
    sleep_ms(50);

    // ── I2C Scanner (temporary debug) ──────────────────────────
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
    stdio_flush();
    // ────────────────────────────────────────────────────────────

    printf("[I2C] Ready on GP%d/GP%d\n", TOF_PIN_SDA, TOF_PIN_SCL);
    stdio_flush();

    // ── Init Camera ──
    blink_n(2, 200);
    printf("[CAM] Initializing...\n");
    stdio_flush();
    myCAM.begin();
    printf("[CAM] Ready\n");
    stdio_flush();
    blink_n(3, 200);

    ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t frame_buf[FRAME_BYTES];
    static uint8_t read_chunk[200];
    
    // ── Init ToF Sensors ──
    blink_n(4, 150);
    printf("[TOF] Initializing sensors...\n");
    stdio_flush();
    if (!init_all_tof()) {
        printf("[WARN] Not all ToF sensors initialized\n");
        stdio_flush();
    }

    // ── Init Phase 4 subsystems ──
    printf("[P4] Initializing button...\n");       stdio_flush(); button_init();
    printf("[P4] Initializing motors...\n");       stdio_flush(); motors_init();
    printf("[P4] Initializing arms...\n");         stdio_flush(); arms_init();
    printf("[P4] Initializing ball handler...\n"); stdio_flush(); ball_handler_init();

    blink_n(5, 100);
    printf("\n[READY] Autonomous run starting (button not wired)\n");
    printf("[READY] Press again at any time for E-STOP\n\n");
    stdio_flush();

    // ── State machine variables ──
    RoverState state = STATE_INIT;
    absolute_time_t state_entered_at = get_absolute_time();
    uint32_t loop_count = 0;
    const uint32_t CAM_INTERVAL = 3;

    VisionResult vision = {false, 0.0f, 0.0f, 0, 0.0f};

    // ── Main control loop ──
    while (true) {
        loop_count++;
        uint32_t loop_start = time_us_32();

        // ----------------------------------------------------
        // GLOBAL: button always works, in any state
        // ----------------------------------------------------
        if (button_pressed_edge()) {
            if (state == STATE_IDLE) {
                printf("[BTN] start pressed -> INIT\n"); stdio_flush();
                state = STATE_INIT;
                state_entered_at = get_absolute_time();
            } else if (state == STATE_ESTOP) {
                printf("[BTN] resume from ESTOP -> INIT\n"); stdio_flush();
                state = STATE_INIT;
                state_entered_at = get_absolute_time();
            } else {
                printf("[BTN] e-stop pressed\n"); stdio_flush();
                enter_estop();
                state = STATE_ESTOP;
                state_entered_at = get_absolute_time();
            }
        }

        // ----------------------------------------------------
        // ALWAYS read ToF (~20 Hz)
        // ----------------------------------------------------
        read_tof();

        // ----------------------------------------------------
        // Camera every Nth loop (~7 Hz). Skip in IDLE/ESTOP/SORT.
        // ----------------------------------------------------
        bool need_vision = (state == STATE_RUN ||
                            state == STATE_CAPTURE ||
                            state == STATE_RECOVER);
        if (need_vision && (loop_count % CAM_INTERVAL == 0)) {
            read_camera_frame(cam, frame_buf, read_chunk);
            vision = process_frame_rgb565(frame_buf);
        }

        // ----------------------------------------------------
        // PER-STATE behavior
        // ----------------------------------------------------
        DriveCommand cmd = {0.0f, 0.0f, MODE_STOP};

        switch (state) {

            case STATE_IDLE: {
                all_stop();
                gpio_put(LED_PIN, ((time_us_32() / 500000) & 1) ? 1 : 0);
                break;
            }

            case STATE_INIT: {
                printf("[STATE] INIT - sweeping arms up\n"); stdio_flush();
                gpio_put(LED_PIN, 1);
                all_stop();
                arms_sweep(0, ARMS_ROM_MAX);
                sleep_ms(300);
                printf("[STATE] -> RUN\n"); stdio_flush();
                state = STATE_RUN;
                state_entered_at = get_absolute_time();
                target_lost_count = 0;
                break;
            }

            case STATE_RUN: {
                CollisionResult collision = get_collision();
                cmd = compute_drive(vision, collision);

                // CHANGE 6: Capture trigger replaced — camera centers ball in TRACK mode,
                // FM ToF physically confirms ball is in arm capture position (45-100mm).
                // FL/FR both clear ensures it is a ball and not another rover.
                if (cmd.mode == MODE_TRACK && collision.ball_in_capture_zone) {
                    printf("[STATE] RUN -> CAPTURE (FM=%umm FL=%umm FR=%umm)\n",
                           collision.fm_mm, collision.fl_mm, collision.fr_mm);
                    stdio_flush();
                    state = STATE_CAPTURE;
                    state_entered_at = get_absolute_time();
                    break;
                }

                apply_drive_command(cmd);
                break;
            }

            case STATE_CAPTURE: {
                // CHANGE 7: CAPTURE no longer drives forward.
                // Ball is already in position confirmed by FM ToF.
                // Stop motors, lower arm onto ball, raise arm to drop ball into funnel.
                // FM readings are ignored during this sequence — arm briefly blocks sensor.
                motors_stop();
                printf("[STATE] CAPTURE - lowering arm\n"); stdio_flush();
                arms_sweep(ARMS_ROM_MAX, ARMS_ROM_MIN);  // lower onto ball
                sleep_ms(200);
                printf("[STATE] CAPTURE - raising arm\n"); stdio_flush();
                arms_sweep(ARMS_ROM_MIN, ARMS_ROM_MAX);  // raise into funnel
                printf("[STATE] CAPTURE -> SORT\n"); stdio_flush();
                state = STATE_SORT;
                state_entered_at = get_absolute_time();
                break;
            }

            case STATE_SORT: {
                motors_stop();
                printf("[STATE] SORT - running ball handler (BLOCKING ~5s)\n");
                stdio_flush();
                ball_handler_run_one();
                printf("[STATE] SORT -> RECOVER\n"); stdio_flush();
                state = STATE_RECOVER;
                state_entered_at = get_absolute_time();
                break;
            }

            case STATE_RECOVER: {
                // CHANGE 8: Removed danger_rear check — no rear ToF, open arena.
                // Pure time-based reverse only.
                cmd.linear_speed = RECOVER_SPEED;
                cmd.turn_rate    = 0.0f;
                cmd.mode         = MODE_REVERSE;
                apply_drive_command(cmd);

                int64_t elapsed_ms =
                    absolute_time_diff_us(state_entered_at, get_absolute_time()) / 1000;
                if (elapsed_ms >= (int64_t)RECOVER_DRIVE_MS) {
                    motors_stop();
                    printf("[STATE] RECOVER -> RUN\n"); stdio_flush();
                    state = STATE_RUN;
                    state_entered_at = get_absolute_time();
                    target_lost_count = 0;
                }
                break;
            }

            case STATE_ESTOP: {
                all_stop();
                gpio_put(LED_PIN, ((time_us_32() / 100000) & 1) ? 1 : 0);
                break;
            }
        }

        // ----------------------------------------------------
        // Periodic debug output
        // ----------------------------------------------------
        if (loop_count % 5 == 0 && state != STATE_IDLE && state != STATE_ESTOP) {
            uint32_t loop_us = time_us_32() - loop_start;
            printf("[%s/%s] spd=%+.2f trn=%+.2f | ",
                   state_name(state), mode_names[cmd.mode],
                   cmd.linear_speed, cmd.turn_rate);

            if (vision.target_found) {
                printf("ball:x=%+.2f sz=%u | ", vision.target_x, vision.target_size);
            } else {
                printf("ball:--- | ");
            }

            // CHANGE 8 (cont): debug label changed from RR to FM
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