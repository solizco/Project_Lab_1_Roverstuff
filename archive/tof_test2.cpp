/*
 * tof_test.cpp
 * Standalone TOF test — Pico 2 (RP2350)
 *
 * Tests all three sensors in isolation with no other subsystems wired.
 * Stripped directly from main.cpp — same init sequence, same read loop,
 * same median filter and zone classification. Only difference is there
 * is no camera, motors, arms, or state machine.
 *
 * Wiring (same as main):
 *   I2C0  SDA = GP4,  SCL = GP5
 *   FL = VL53L0X  XSHUT = GP8   → reassigned to 0x30
 *   FR = VL53L0X  XSHUT = GP9   → reassigned to 0x31
 *   FM = VL53L1X  XSHUT = GP7   → reassigned to 0x32
 *
 * Expected serial output (115200 baud via USB):
 *   [TOF] FL: OK at 0x30
 *   [TOF] FR: OK at 0x31
 *   [TOF] FM: OK at 0x32
 *   [TOF] 3/3 sensors ready
 *   FL:  342[CLR ]  FR:  287[CLR ]  FM:   78[SLOW]
 *   ...
 */

#include <cstdio>
#include <cstring>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "VL53L1X.h"

// ── Pin definitions ──────────────────────────────────────────────────────────
// Copy these from your config_pins.h if they differ
#define TOF_I2C           i2c0
#define TOF_PIN_SDA       4
#define TOF_PIN_SCL       5
#define TOF_PIN_XSHUT_FL  8
#define TOF_PIN_XSHUT_FR  6
#define TOF_PIN_XSHUT_FM  7
#define LED_PIN           25

// ── Sensor addresses after reassignment ─────────────────────────────────────
#define NUM_SENSORS 3
static const uint8_t SENSOR_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32};
static const char*   SENSOR_NAMES[NUM_SENSORS]  = {"FL", "FR", "FM"};

// ── Distance zone thresholds (same as main) ──────────────────────────────────
#define STOP_DISTANCE_MM  150
#define SLOW_DISTANCE_MM  250

typedef enum { ZONE_STOP, ZONE_SLOW, ZONE_CLEAR, ZONE_ERROR } DistanceZone;
static const char* zone_names[] = {"STOP", "SLOW", "CLR ", "ERR "};

// ── Median filter (same as main) ─────────────────────────────────────────────
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

// ── TwoWire shim instance and sensor objects ─────────────────────────────────
static TwoWire Wire0(TOF_I2C);

static VL53L0X sensorFL;
static VL53L0X sensorFR;
static VL53L1X sensorFM;

// ── Zone classifier ──────────────────────────────────────────────────────────
static DistanceZone classify(uint16_t mm) {
    if (mm == 0)                  return ZONE_ERROR;
    if (mm < STOP_DISTANCE_MM)    return ZONE_STOP;
    if (mm < SLOW_DISTANCE_MM)    return ZONE_SLOW;
    return ZONE_CLEAR;
}

// ── TOF init ──────────────────────────────────────────────────────────────────
static bool init_all_tof() {
    int ok = 0;

    gpio_put(TOF_PIN_XSHUT_FL, 0);
    gpio_put(TOF_PIN_XSHUT_FR, 0);
    gpio_put(TOF_PIN_XSHUT_FM, 0);
    sleep_ms(20);

    // ── FL: VL53L0X ──
    gpio_put(TOF_PIN_XSHUT_FL, 1);
    sleep_ms(100);

    uint8_t dummy;
    int raw = i2c_read_timeout_us(TOF_I2C, 0x29, &dummy, 1, false, 20000);
    printf("[TOF] FL: raw scan at 0x29 = %d %s\n", raw,
           raw >= 0 ? "(sensor found)" : "(nothing - check wiring)");
    stdio_flush();

    sensorFL.setBus(&Wire0);
    sensorFL.setTimeout(500);
    if (!sensorFL.init()) {
        printf("[TOF] FL: init() failed\n"); stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FL, 0);
    } else {
        sensorFL.setAddress(SENSOR_ADDRS[0]);
        sleep_ms(20);
        // startContinuous moved to end — do NOT call here
        tof[0].initialized = true;
        mf_init(&tof[0].filter);
        printf("[TOF] FL: OK at 0x%02X\n", SENSOR_ADDRS[0]); stdio_flush();
        ok++;
    }

    // ── FR: VL53L0X ──
    bool fr_ok = false;
    for (int attempt = 1; attempt <= 3 && !fr_ok; attempt++) {
        gpio_put(TOF_PIN_XSHUT_FR, 0);
        sleep_ms(20);
        gpio_put(TOF_PIN_XSHUT_FR, 1);
        sleep_ms(100);

        raw = i2c_read_timeout_us(TOF_I2C, 0x29, &dummy, 1, false, 20000);
        printf("[TOF] FR: attempt %d raw scan = %d %s\n", attempt, raw,
            raw >= 0 ? "(sensor found)" : "(nothing)");
        stdio_flush();

        if (raw < 0) continue;

        sensorFR.setBus(&Wire0);
        sensorFR.setTimeout(500);
        if (!sensorFR.init()) {
            printf("[TOF] FR: init() failed on attempt %d\n", attempt);
            stdio_flush();
        } else {
            sensorFR.setAddress(SENSOR_ADDRS[1]);
            sleep_ms(20);
            tof[1].initialized = true;
            mf_init(&tof[1].filter);
            printf("[TOF] FR: OK at 0x%02X\n", SENSOR_ADDRS[1]); stdio_flush();
            ok++;
            fr_ok = true;
        }
    }
    if (!fr_ok) {
        printf("[TOF] FR: all attempts failed\n"); stdio_flush();
        gpio_put(TOF_PIN_XSHUT_FR, 0);
    }

    // ── FM: VL53L1X ──
    gpio_put(TOF_PIN_XSHUT_FM, 1);
    sleep_ms(100);

    raw = i2c_read_timeout_us(TOF_I2C, 0x29, &dummy, 1, false, 20000);
    printf("[TOF] FM: raw scan at 0x29 = %d %s\n", raw,
           raw >= 0 ? "(sensor found)" : "(nothing - check wiring)");
    stdio_flush();

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
        // startContinuous moved to end — do NOT call here
        tof[2].initialized = true;
        mf_init(&tof[2].filter);
        printf("[TOF] FM: OK at 0x%02X\n", SENSOR_ADDRS[2]); stdio_flush();
        ok++;
    }

    // ── Start continuous AFTER all three sensors are initialized ──────────────
    // FL's continuous ranging was previously started before FR was even released
    // from reset, causing its I2C traffic to corrupt FR's init sequence.
    // All address assignments must be complete before any sensor starts ranging.
    printf("[TOF] Starting continuous ranging...\n"); stdio_flush();
    if (tof[0].initialized) sensorFL.startContinuous(0);
    if (tof[1].initialized) sensorFR.startContinuous(0);
    if (tof[2].initialized) sensorFM.startContinuous(55);

    printf("[TOF] %d/3 sensors ready\n", ok); stdio_flush();
    return (ok == 3);
}

// ── Read all sensors ──────────────────────────────────────────────────────────
static void read_tof() {
    // FL
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

    // FR
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

    // FM — non-blocking poll
    if (tof[2].initialized) {
        if (sensorFM.dataReady()) {
            uint16_t d = sensorFM.read(false);
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

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    // Drive XSHUT pins LOW immediately — before USB wait —
    // so sensors don't all boot at 0x29 and collide
    gpio_init(TOF_PIN_XSHUT_FL); gpio_set_dir(TOF_PIN_XSHUT_FL, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FL, 0);
    gpio_init(TOF_PIN_XSHUT_FR); gpio_set_dir(TOF_PIN_XSHUT_FR, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FR, 0);
    gpio_init(TOF_PIN_XSHUT_FM); gpio_set_dir(TOF_PIN_XSHUT_FM, GPIO_OUT); gpio_put(TOF_PIN_XSHUT_FM, 0);

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Wait up to 3s for USB, then continue anyway
    uint32_t usb_start = time_us_32();
    while (!stdio_usb_connected()) {
        if ((time_us_32() - usb_start) > 3000000) break;
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(200);

    printf("\n============================\n");
    printf("  TOF Sensor Test\n");
    printf("  FL/FR = VL53L0X  FM = VL53L1X\n");
    printf("============================\n\n");
    stdio_flush();

    // I2C init — pins first, then begin
    gpio_set_function(TOF_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_PIN_SDA);
    gpio_pull_up(TOF_PIN_SCL);
    sleep_ms(10);
    Wire0.begin(100000);
    sleep_ms(50);

    // Initial bus scan before any XSHUT is released
    printf("[I2C] Pre-init scan (all XSHUT low, expect nothing)...\n");
    stdio_flush();
    bool found = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        uint8_t dummy;
        if (i2c_read_timeout_us(TOF_I2C, a, &dummy, 1, false, 10000) >= 0) {
            printf("[I2C] Found 0x%02X\n", a); stdio_flush();
            found = true;
        }
    }
    if (!found) printf("[I2C] Nothing found (correct)\n");
    stdio_flush();

    // Init sensors
    printf("\n[TOF] Initializing...\n"); stdio_flush();
    bool all_ok = init_all_tof();
    if (!all_ok) {
        printf("[WARN] Not all sensors initialized — continuing anyway\n");
        stdio_flush();
    }

    // Post-init bus scan to confirm addresses
    printf("\n[I2C] Post-init scan (expect 0x30, 0x31, 0x32)...\n");
    stdio_flush();
    for (uint8_t a = 0x08; a < 0x78; a++) {
        uint8_t dummy;
        if (i2c_read_timeout_us(TOF_I2C, a, &dummy, 1, false, 10000) >= 0) {
            printf("[I2C] Found 0x%02X\n", a); stdio_flush();
        }
    }

    printf("\n[LOOP] Reading sensors every 100ms...\n");
    printf("       ZONE key: STOP=<%dmm  SLOW=<%dmm  CLR=above  ERR=no data\n\n",
           STOP_DISTANCE_MM, SLOW_DISTANCE_MM);
    stdio_flush();

    uint32_t loop = 0;
    while (true) {
        loop++;
        read_tof();

        printf("FL:%5u[%s]  FR:%5u[%s]  FM:%5u[%s]\n",
               tof[0].filtered_mm, zone_names[tof[0].zone],
               tof[1].filtered_mm, zone_names[tof[1].zone],
               tof[2].filtered_mm, zone_names[tof[2].zone]);
        stdio_flush();

        gpio_put(LED_PIN, (loop & 1) ? 1 : 0);

        sleep_ms(100);
    }
}