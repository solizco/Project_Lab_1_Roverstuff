/*
 * Rover Phase 2 — VL53L1X ToF Sensor Bring-Up
 * Board:  Pico 2 (RP2350)
 * Sensors: 3× VL53L1X (I2C) via TOF400C boards
 *
 * Wiring:
 *   All sensors share I2C0: SDA=GP4, SCL=GP5
 *   Front Left  SHUT → GP6
 *   Front Right SHUT → GP7
 *   Rear        SHUT → GP8
 *   All VIN → 3V3, all GND → GND, INT not connected
 *
 * XSHUT address assignment sequence:
 *   1. Hold all SHUT pins LOW (sensors in reset)
 *   2. Bring up sensor 0 (SHUT HIGH), init it, assign address 0x30
 *   3. Bring up sensor 1, init it, assign address 0x31
 *   4. Bring up sensor 2, init it, assign address 0x32
 *
 * LED patterns:
 *   Fast blink = waiting for USB
 *   1 blink    = starting init
 *   3 blinks   = all sensors initialized
 *   Steady     = running
 *   Slow blink = error
 */

#include <cstdio>
#include <cstring>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

extern "C" {
#include "VL53L1X_api.h"
}

// ── Pin definitions ──────────────────────────────────────────────────────────
#define I2C_PORT    i2c0
#define I2C_SDA     4
#define I2C_SCL     5

#define XSHUT_FRONT_LEFT   6
#define XSHUT_FRONT_RIGHT  7
#define XSHUT_REAR         8

#define LED_PIN     25

// ── Sensor configuration ─────────────────────────────────────────────────────
#define NUM_SENSORS 3
#define VL53L1X_DEFAULT_ADDR  0x29  // 7-bit default address

static const uint16_t SENSOR_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32};
static const uint8_t  XSHUT_PINS[NUM_SENSORS]   = {XSHUT_FRONT_LEFT, XSHUT_FRONT_RIGHT, XSHUT_REAR};
static const char*    SENSOR_NAMES[NUM_SENSORS]  = {"FrontL", "FrontR", "Rear  "};

// ── Distance zones (in mm) ───────────────────────────────────────────────────
#define STOP_DISTANCE_MM   150
#define SLOW_DISTANCE_MM   400

typedef enum {
    ZONE_STOP,
    ZONE_SLOW,
    ZONE_CLEAR,
    ZONE_ERROR
} DistanceZone;

static const char* zone_names[] = {"STOP ", "SLOW ", "CLEAR", "ERROR"};

// ── Median filter ────────────────────────────────────────────────────────────
#define MEDIAN_WINDOW 5

typedef struct {
    uint16_t buffer[MEDIAN_WINDOW];
    uint8_t  index;
    uint8_t  count;
} MedianFilter;

static void median_filter_init(MedianFilter* f) {
    memset(f->buffer, 0, sizeof(f->buffer));
    f->index = 0;
    f->count = 0;
}

static void median_filter_add(MedianFilter* f, uint16_t value) {
    f->buffer[f->index] = value;
    f->index = (f->index + 1) % MEDIAN_WINDOW;
    if (f->count < MEDIAN_WINDOW) f->count++;
}

static uint16_t median_filter_get(MedianFilter* f) {
    if (f->count == 0) return 0;
    uint16_t sorted[MEDIAN_WINDOW];
    memcpy(sorted, f->buffer, sizeof(uint16_t) * f->count);
    std::sort(sorted, sorted + f->count);
    return sorted[f->count / 2];
}

// ── Sensor state ─────────────────────────────────────────────────────────────
typedef struct {
    uint16_t     addr;
    uint16_t     raw_mm;
    uint16_t     filtered_mm;
    DistanceZone zone;
    bool         initialized;
    MedianFilter filter;
} ToFSensor;

static ToFSensor sensors[NUM_SENSORS];

// ── Collision result ─────────────────────────────────────────────────────────
typedef struct {
    bool         danger_left;
    bool         danger_right;
    bool         danger_rear;
    bool         front_blocked;
    DistanceZone front_left_zone;
    DistanceZone front_right_zone;
    DistanceZone rear_zone;
    uint16_t     front_left_mm;
    uint16_t     front_right_mm;
    uint16_t     rear_mm;
} CollisionResult;

// ── Helpers ──────────────────────────────────────────────────────────────────
static void blink_n(int n, uint32_t ms) {
    for (int i = 0; i < n; i++) {
        gpio_put(LED_PIN, 1); sleep_ms(ms);
        gpio_put(LED_PIN, 0); sleep_ms(ms);
    }
}

static DistanceZone classify_distance(uint16_t dist_mm) {
    if (dist_mm == 0)                  return ZONE_ERROR;
    if (dist_mm < STOP_DISTANCE_MM)    return ZONE_STOP;
    if (dist_mm < SLOW_DISTANCE_MM)    return ZONE_SLOW;
    return ZONE_CLEAR;
}

// ── I2C scan ─────────────────────────────────────────────────────────────────
static void i2c_scan() {
    printf("[I2C] Scanning bus...\n");
    int found = 0;
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_timeout_us(I2C_PORT, addr, &dummy, 1, false, 10000);
        if (ret >= 0) {
            printf("[I2C] Found device at 0x%02X\n", addr);
            found++;
        }
    }
    printf("[I2C] Scan complete, %d devices found\n", found);
    stdio_flush();
}

// ── Initialize all sensors with unique addresses ──────────────────────────────
static bool init_all_sensors() {
    // Hold all XSHUT pins LOW — all sensors in reset
    for (int i = 0; i < NUM_SENSORS; i++) {
        gpio_init(XSHUT_PINS[i]);
        gpio_set_dir(XSHUT_PINS[i], GPIO_OUT);
        gpio_put(XSHUT_PINS[i], 0);
    }
    sleep_ms(50);

    printf("[TOF] All sensors held in reset\n");
    stdio_flush();

    // Bring up each sensor one at a time
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].initialized = false;
        sensors[i].addr = SENSOR_ADDRS[i];
        median_filter_init(&sensors[i].filter);

        // Release this sensor from reset
        gpio_put(XSHUT_PINS[i], 1);
        sleep_ms(10);  // VL53L1X boot time is ~1.2ms, 10ms is plenty

        printf("[TOF] %s XSHUT released, waiting for boot...\n", SENSOR_NAMES[i]);
        stdio_flush();

        // Re-register I2C with library for default address communication
        VL53L1X_I2C_Init(VL53L1X_DEFAULT_ADDR, I2C_PORT);

        // Wait for boot state
        uint8_t boot_state = 0;
        int retries = 0;
        while (boot_state == 0 && retries < 50) {
            int8_t status = VL53L1X_BootState(VL53L1X_DEFAULT_ADDR, &boot_state);
            if (status != 0) {
                printf("[TOF] %s BootState I2C error (status=%d), retry %d\n",
                       SENSOR_NAMES[i], status, retries);
                stdio_flush();
            }
            sleep_ms(10);
            retries++;
        }

        if (boot_state == 0) {
            printf("[TOF] ERROR: %s failed to boot after %d retries\n",
                   SENSOR_NAMES[i], retries);
            stdio_flush();
            continue;
        }

        printf("[TOF] %s booted OK (retries=%d)\n", SENSOR_NAMES[i], retries);
        stdio_flush();

        // Sensor init
        int8_t status = VL53L1X_SensorInit(VL53L1X_DEFAULT_ADDR);
        if (status != 0) {
            printf("[TOF] ERROR: %s SensorInit failed (status=%d)\n",
                   SENSOR_NAMES[i], status);
            stdio_flush();
            continue;
        }

        printf("[TOF] %s SensorInit OK\n", SENSOR_NAMES[i]);
        stdio_flush();

        // Assign new unique address (library expects 8-bit shifted address)
        status = VL53L1X_SetI2CAddress(VL53L1X_DEFAULT_ADDR, SENSOR_ADDRS[i] << 1);
        if (status != 0) {
            printf("[TOF] ERROR: %s SetI2CAddress failed (status=%d)\n",
                   SENSOR_NAMES[i], status);
            stdio_flush();
            continue;
        }

        printf("[TOF] %s assigned address 0x%02X\n", SENSOR_NAMES[i], SENSOR_ADDRS[i]);
        stdio_flush();

        // Configure: long distance mode, 50ms timing budget
        VL53L1X_SetDistanceMode(SENSOR_ADDRS[i], 2);
        VL53L1X_SetTimingBudgetInMs(SENSOR_ADDRS[i], 50);
        VL53L1X_SetInterMeasurementInMs(SENSOR_ADDRS[i], 55);

        // Start ranging
        status = VL53L1X_StartRanging(SENSOR_ADDRS[i]);
        if (status != 0) {
            printf("[TOF] ERROR: %s StartRanging failed (status=%d)\n",
                   SENSOR_NAMES[i], status);
            stdio_flush();
            continue;
        }

        sensors[i].initialized = true;
        printf("[TOF] %s ranging!\n\n", SENSOR_NAMES[i]);
        stdio_flush();
    }

    // Final bus scan to confirm addresses
    printf("[TOF] Post-init I2C scan:\n");
    i2c_scan();
    printf("\n");

    int ok_count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensors[i].initialized) ok_count++;
    }

    printf("[TOF] %d / %d sensors initialized\n", ok_count, NUM_SENSORS);
    stdio_flush();
    return ok_count > 0;
}

// ── Read all sensors ──────────────────────────────────────────────────────────
static void read_all_sensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!sensors[i].initialized) {
            sensors[i].zone = ZONE_ERROR;
            continue;
        }

        uint8_t data_ready = 0;
        VL53L1X_CheckForDataReady(sensors[i].addr, &data_ready);

        if (data_ready) {
            uint16_t distance = 0;
            VL53L1X_GetDistance(sensors[i].addr, &distance);
            VL53L1X_ClearInterrupt(sensors[i].addr);

            if (distance > 0 && distance < 8000) {
                sensors[i].raw_mm = distance;
                median_filter_add(&sensors[i].filter, distance);
                sensors[i].filtered_mm = median_filter_get(&sensors[i].filter);
            }
        }

        sensors[i].zone = classify_distance(sensors[i].filtered_mm);
    }
}

// ── Compute collision decision ────────────────────────────────────────────────
static CollisionResult compute_collision() {
    CollisionResult result;

    result.front_left_mm   = sensors[0].filtered_mm;
    result.front_right_mm  = sensors[1].filtered_mm;
    result.rear_mm         = sensors[2].filtered_mm;

    result.front_left_zone  = sensors[0].zone;
    result.front_right_zone = sensors[1].zone;
    result.rear_zone        = sensors[2].zone;

    result.danger_left  = (sensors[0].zone == ZONE_STOP);
    result.danger_right = (sensors[1].zone == ZONE_STOP);
    result.danger_rear  = (sensors[2].zone == ZONE_STOP);

    uint16_t front_min = sensors[0].filtered_mm;
    if (sensors[1].filtered_mm < front_min) front_min = sensors[1].filtered_mm;
    result.front_blocked = (front_min > 0 && front_min < STOP_DISTANCE_MM);

    return result;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Wait for USB CDC — mandatory on Pico 2, data is lost without this
    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n========================================\n");
    printf("  Rover Phase 2 — ToF Sensor Test\n");
    printf("  Board: Pico 2 (RP2350)\n");
    printf("  Sensors: 3x VL53L1X\n");
    printf("========================================\n\n");
    stdio_flush();

    // Initialize I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    printf("[I2C] Initialized at 400kHz on SDA=GP%d SCL=GP%d\n\n", I2C_SDA, I2C_SCL);
    stdio_flush();

    // Register I2C bus with VL53L1X library BEFORE any library calls
    VL53L1X_I2C_Init(VL53L1X_DEFAULT_ADDR, I2C_PORT);

    // Scan before init
    printf("[PRE-INIT] ");
    i2c_scan();
    printf("\n");

    blink_n(1, 300);

    if (!init_all_sensors()) {
        printf("[FATAL] No sensors initialized. Check wiring.\n");
        stdio_flush();
        while (true) {
            gpio_put(LED_PIN, 1); sleep_ms(1000);
            gpio_put(LED_PIN, 0); sleep_ms(1000);
        }
    }

    blink_n(3, 200);

    printf("\n[READY] Reading sensors.\n");
    printf("[READY] Zones: STOP < %dmm, SLOW < %dmm, CLEAR >= %dmm\n\n",
           STOP_DISTANCE_MM, SLOW_DISTANCE_MM, SLOW_DISTANCE_MM);
    stdio_flush();

    uint32_t loop_count = 0;

    while (true) {
        gpio_put(LED_PIN, 1);

        read_all_sensors();
        CollisionResult collision = compute_collision();

        if (loop_count % 5 == 0) {
            printf("FL:%4umm [%s]  FR:%4umm [%s]  RR:%4umm [%s]",
                   sensors[0].filtered_mm, zone_names[sensors[0].zone],
                   sensors[1].filtered_mm, zone_names[sensors[1].zone],
                   sensors[2].filtered_mm, zone_names[sensors[2].zone]);

            if (collision.front_blocked) printf("  ** FRONT BLOCKED **");
            if (collision.danger_rear)   printf("  ** REAR DANGER **");

            printf("\n");
            stdio_flush();
        }

        gpio_put(LED_PIN, 0);
        loop_count++;
        sleep_ms(50);
    }
}