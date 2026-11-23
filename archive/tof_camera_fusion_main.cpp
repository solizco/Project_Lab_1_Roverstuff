/*
 * Rover Phase 3 — Sensor Fusion: Camera + ToF
 * Board:  Pico 2 (RP2350)
 * Camera: Arducam Mega 3MP (SPI) — 128×128 RGB565 red detection
 * ToF:    3× VL53L1X (I2C) — front left, front right, rear
 *
 * This merges Phase 1 (vision) and Phase 2 (ToF) into a single
 * non-blocking control loop with a state machine.
 *
 * Control priority:
 *   1. ToF SAFETY always wins — obstacles override camera
 *   2. Camera VISION steers toward target when path is clear
 *   3. SEARCH behavior when target is lost
 *   4. RECOVERY when stuck or target lost too long
 *
 * Drive modes:
 *   SEARCH  — no target, slow rotate to find one
 *   TRACK   — target found, steer toward it
 *   AVOID   — obstacle detected, steer away
 *   REVERSE — front blocked, back up
 *   STOP    — rear blocked during reverse, full stop
 *
 * Output each loop:
 *   DriveCommand { linear_speed, turn_rate, mode }
 *   These will connect to motors in Phase 4.
 *
 * LED patterns:
 *   Fast blink = waiting for USB
 *   2 blinks = camera init
 *   3 blinks = camera OK
 *   4 blinks = ToF init
 *   5 blinks = all systems go
 *   Steady flicker = running
 */

#include <cstdio>
#include <cstring>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "Arducam_Mega.h"
#include "ArducamCamera.h"

extern "C" {
#include "VL53L1X_api.h"
}

// ═════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═════════════════════════════════════════════════════════════════════════════

// Camera SPI
#define CAM_CS_PIN   17
// Camera uses SPI0: MISO=GP16, MOSI=GP19, SCK=GP18 (handled by SDK)

// ToF I2C
#define I2C_PORT     i2c0
#define I2C_SDA      4
#define I2C_SCL      5

// ToF XSHUT
#define XSHUT_FL     6   // Front Left
#define XSHUT_FR     7   // Front Right
#define XSHUT_RR     8   // Rear

#define LED_PIN      25

// ═════════════════════════════════════════════════════════════════════════════
//  CAMERA VISION (from Phase 1)
// ═════════════════════════════════════════════════════════════════════════════

#define FRAME_W 128
#define FRAME_H 128
#define FRAME_PIXELS (FRAME_W * FRAME_H)
#define FRAME_BYTES  (FRAME_PIXELS * 2)

static Arducam_Mega myCAM(CAM_CS_PIN);

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
    // Drain extra
    while (bytesRead < total) {
        uint32_t remaining = total - bytesRead;
        uint8_t chunk = (remaining > 200) ? 200 : (uint8_t)remaining;
        readBuff(cam, read_chunk, chunk);
        bytesRead += chunk;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  TOF COLLISION AVOIDANCE (from Phase 2)
// ═════════════════════════════════════════════════════════════════════════════

#define NUM_SENSORS 3
#define VL53L1X_DEFAULT_ADDR 0x29

static const uint16_t SENSOR_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32};
static const uint8_t  XSHUT_PINS[NUM_SENSORS]   = {XSHUT_FL, XSHUT_FR, XSHUT_RR};
static const char*    SENSOR_NAMES[NUM_SENSORS]  = {"FL", "FR", "RR"};

#define STOP_DISTANCE_MM  150
#define SLOW_DISTANCE_MM  400

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
    uint16_t     addr;
    uint16_t     raw_mm;
    uint16_t     filtered_mm;
    DistanceZone zone;
    bool         initialized;
    MedianFilter filter;
} ToFSensor;

static ToFSensor tof[NUM_SENSORS];

static DistanceZone classify(uint16_t mm) {
    if (mm == 0) return ZONE_ERROR;
    if (mm < STOP_DISTANCE_MM) return ZONE_STOP;
    if (mm < SLOW_DISTANCE_MM) return ZONE_SLOW;
    return ZONE_CLEAR;
}

struct CollisionResult {
    bool danger_left;
    bool danger_right;
    bool danger_rear;
    bool front_blocked;
    DistanceZone fl_zone, fr_zone, rr_zone;
    uint16_t fl_mm, fr_mm, rr_mm;
};

static bool init_tof_sensor(int idx) {
    tof[idx].initialized = false;
    tof[idx].addr = SENSOR_ADDRS[idx];
    mf_init(&tof[idx].filter);

    gpio_put(XSHUT_PINS[idx], 1);
    sleep_ms(100);

    uint8_t dummy;
    if (i2c_read_timeout_us(I2C_PORT, VL53L1X_DEFAULT_ADDR, &dummy, 1, false, 50000) < 0) {
        printf("[TOF] %s: not found\n", SENSOR_NAMES[idx]);
        stdio_flush();
        return false;
    }

    uint8_t boot = 0;
    for (int r = 0; r < 50 && boot == 0; r++) {
        VL53L1X_BootState(VL53L1X_DEFAULT_ADDR, &boot);
        sleep_ms(20);
    }
    if (!boot) { printf("[TOF] %s: boot fail\n", SENSOR_NAMES[idx]); stdio_flush(); return false; }

    if (VL53L1X_SensorInit(VL53L1X_DEFAULT_ADDR)) {
        printf("[TOF] %s: init fail\n", SENSOR_NAMES[idx]); stdio_flush(); return false;
    }

    VL53L1X_SetI2CAddress(VL53L1X_DEFAULT_ADDR, SENSOR_ADDRS[idx] << 1);
    VL53L1X_SetDistanceMode(SENSOR_ADDRS[idx], 2);
    VL53L1X_SetTimingBudgetInMs(SENSOR_ADDRS[idx], 50);
    VL53L1X_SetInterMeasurementInMs(SENSOR_ADDRS[idx], 55);
    VL53L1X_StartRanging(SENSOR_ADDRS[idx]);

    tof[idx].initialized = true;
    printf("[TOF] %s: OK at 0x%02X\n", SENSOR_NAMES[idx], SENSOR_ADDRS[idx]);
    stdio_flush();
    return true;
}

static bool init_all_tof() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        gpio_init(XSHUT_PINS[i]);
        gpio_set_dir(XSHUT_PINS[i], GPIO_OUT);
        gpio_put(XSHUT_PINS[i], 0);
    }
    sleep_ms(100);

    int ok = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (init_tof_sensor(i)) ok++;
    }
    printf("[TOF] %d/%d sensors ready\n", ok, NUM_SENSORS);
    stdio_flush();
    return ok >= 2;  // Need at least 2 front sensors
}

static void read_tof() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!tof[i].initialized) { tof[i].zone = ZONE_ERROR; continue; }

        uint8_t ready = 0;
        VL53L1X_CheckForDataReady(tof[i].addr, &ready);
        if (ready) {
            uint16_t d = 0;
            VL53L1X_GetDistance(tof[i].addr, &d);
            VL53L1X_ClearInterrupt(tof[i].addr);
            if (d > 0 && d < 8000) {
                tof[i].raw_mm = d;
                mf_add(&tof[i].filter, d);
                tof[i].filtered_mm = mf_get(&tof[i].filter);
            }
        }
        tof[i].zone = classify(tof[i].filtered_mm);
    }
}

static CollisionResult get_collision() {
    CollisionResult c;
    c.fl_mm = tof[0].filtered_mm;
    c.fr_mm = tof[1].filtered_mm;
    c.rr_mm = tof[2].filtered_mm;
    c.fl_zone = tof[0].zone;
    c.fr_zone = tof[1].zone;
    c.rr_zone = tof[2].zone;
    c.danger_left  = (tof[0].zone == ZONE_STOP);
    c.danger_right = (tof[1].zone == ZONE_STOP);
    c.danger_rear  = (tof[2].zone == ZONE_STOP);
    uint16_t front_min = std::min(c.fl_mm, c.fr_mm);
    c.front_blocked = (front_min > 0 && front_min < STOP_DISTANCE_MM);
    return c;
}

// ═════════════════════════════════════════════════════════════════════════════
//  DRIVE COMMAND & STATE MACHINE
// ═════════════════════════════════════════════════════════════════════════════

typedef enum {
    MODE_SEARCH,    // No target — slow rotate
    MODE_TRACK,     // Target found — steer toward it
    MODE_AVOID,     // Obstacle on one side — steer away
    MODE_REVERSE,   // Front blocked — back up
    MODE_STOP       // Rear blocked during reverse — full stop
} DriveMode;

static const char* mode_names[] = {"SEARCH", "TRACK ", "AVOID ", "REVRSE", "STOP  "};

struct DriveCommand {
    float     linear_speed;   // -1.0 (full reverse) to +1.0 (full forward)
    float     turn_rate;      // -1.0 (hard left) to +1.0 (hard right)
    DriveMode mode;
};

// Speed constants (tune these for your motors)
static const float SPEED_FULL    = 0.8f;
static const float SPEED_SLOW    = 0.4f;
static const float SPEED_SEARCH  = 0.3f;
static const float SPEED_REVERSE = -0.4f;
static const float TURN_TRACK    = 0.6f;   // How aggressively to steer toward target
static const float TURN_AVOID    = 0.7f;   // How aggressively to steer away from obstacle
static const float TURN_SEARCH   = 0.5f;   // Search rotation speed

// Target lost counter for recovery
static uint32_t target_lost_count = 0;
static const uint32_t SEARCH_TIMEOUT = 100;  // ~5 seconds at 20Hz before more aggressive search

static DriveCommand compute_drive(const VisionResult& vision, const CollisionResult& collision) {
    DriveCommand cmd = {0.0f, 0.0f, MODE_STOP};

    // ── Priority 1: Front blocked → reverse ──────────────────────────────
    if (collision.front_blocked) {
        if (collision.danger_rear) {
            // Boxed in — full stop
            cmd.mode = MODE_STOP;
            cmd.linear_speed = 0.0f;
            cmd.turn_rate = 0.0f;
        } else {
            // Back up and turn
            cmd.mode = MODE_REVERSE;
            cmd.linear_speed = SPEED_REVERSE;
            // Turn away from whichever side is closer
            if (collision.fl_mm < collision.fr_mm) {
                cmd.turn_rate = TURN_AVOID;   // Turn right (away from left obstacle)
            } else {
                cmd.turn_rate = -TURN_AVOID;  // Turn left (away from right obstacle)
            }
        }
        target_lost_count = 0;
        return cmd;
    }

    // ── Priority 2: Obstacle on one side → steer away ────────────────────
    if (collision.danger_left && !collision.danger_right) {
        cmd.mode = MODE_AVOID;
        cmd.linear_speed = SPEED_SLOW;
        cmd.turn_rate = TURN_AVOID;   // Turn right
        return cmd;
    }
    if (collision.danger_right && !collision.danger_left) {
        cmd.mode = MODE_AVOID;
        cmd.linear_speed = SPEED_SLOW;
        cmd.turn_rate = -TURN_AVOID;  // Turn left
        return cmd;
    }

    // ── Priority 3: Camera tracking ──────────────────────────────────────
    if (vision.target_found) {
        target_lost_count = 0;
        cmd.mode = MODE_TRACK;

        // Steer toward target
        cmd.turn_rate = vision.target_x * TURN_TRACK;

        // Speed based on proximity (more red pixels = closer = slower approach)
        if (vision.target_size > 500) {
            cmd.linear_speed = SPEED_SLOW;   // Close — slow down
        } else {
            cmd.linear_speed = SPEED_FULL;   // Far — full speed
        }

        // Slow down if either front sensor is in SLOW zone
        if (tof[0].zone == ZONE_SLOW || tof[1].zone == ZONE_SLOW) {
            cmd.linear_speed = SPEED_SLOW;
        }

        return cmd;
    }

    // ── Priority 4: No target → search ───────────────────────────────────
    target_lost_count++;
    cmd.mode = MODE_SEARCH;
    cmd.linear_speed = 0.0f;

    if (target_lost_count > SEARCH_TIMEOUT) {
        // Aggressive search — alternate direction
        cmd.turn_rate = ((target_lost_count / 50) % 2 == 0) ? TURN_SEARCH : -TURN_SEARCH;
        cmd.linear_speed = SPEED_SLOW;  // Move while searching
    } else {
        // Normal search — slow rotate in one direction
        cmd.turn_rate = TURN_SEARCH;
    }

    return cmd;
}

// ═════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═════════════════════════════════════════════════════════════════════════════

static void blink_n(int n, uint32_t ms) {
    for (int i = 0; i < n; i++) {
        gpio_put(LED_PIN, 1); sleep_ms(ms);
        gpio_put(LED_PIN, 0); sleep_ms(ms);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  MAIN
// ═════════════════════════════════════════════════════════════════════════════

int main()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Wait for USB CDC
    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n========================================\n");
    printf("  Rover Phase 3 — Sensor Fusion\n");
    printf("  Camera + ToF + State Machine\n");
    printf("========================================\n\n");
    stdio_flush();

    // ── Init I2C for ToF sensors ─────────────────────────────────────────
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    VL53L1X_I2C_Init(VL53L1X_DEFAULT_ADDR, I2C_PORT);

    printf("[I2C] Ready on GP%d/GP%d\n", I2C_SDA, I2C_SCL);
    stdio_flush();

    // ── Init Camera ──────────────────────────────────────────────────────
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

    // ── Init ToF Sensors ─────────────────────────────────────────────────
    blink_n(4, 150);
    printf("[TOF] Initializing sensors...\n");
    stdio_flush();

    if (!init_all_tof()) {
        printf("[WARN] Not all ToF sensors initialized — running with what we have\n");
        stdio_flush();
    }

    blink_n(5, 100);

    printf("\n[READY] Fusion control loop starting\n");
    printf("[READY] Modes: SEARCH | TRACK | AVOID | REVERSE | STOP\n\n");
    stdio_flush();

    // ── Main control loop ────────────────────────────────────────────────
    // Camera runs every N loops (slower), ToF runs every loop (faster)
    uint32_t loop_count = 0;
    const uint32_t CAM_INTERVAL = 3;  // Camera every 3rd loop

    VisionResult vision = {false, 0.0f, 0.0f, 0, 0.0f};

    while (true) {
        gpio_put(LED_PIN, 1);
        uint32_t loop_start = time_us_32();

        // ── Read ToF every loop (~20Hz) ──────────────────────────────────
        read_tof();
        CollisionResult collision = get_collision();

        // ── Read camera every Nth loop (~7Hz) ────────────────────────────
        if (loop_count % CAM_INTERVAL == 0) {
            read_camera_frame(cam, frame_buf, read_chunk);
            vision = process_frame_rgb565(frame_buf);
        }

        // ── Compute drive command ────────────────────────────────────────
        DriveCommand cmd = compute_drive(vision, collision);

        // ── Phase 4: This is where motor output will go ──────────────────
        // motor_set(cmd.linear_speed, cmd.turn_rate);

        // ── Debug output (every 5th loop) ────────────────────────────────
        if (loop_count % 5 == 0) {
            uint32_t loop_us = time_us_32() - loop_start;

            printf("[%s] spd=%+.2f trn=%+.2f | ",
                   mode_names[cmd.mode], cmd.linear_speed, cmd.turn_rate);

            if (vision.target_found) {
                printf("ball:x=%+.2f sz=%u | ", vision.target_x, vision.target_size);
            } else {
                printf("ball:--- | ");
            }

            printf("FL:%4u[%s] FR:%4u[%s] RR:%4u[%s] | %lums\n",
                   tof[0].filtered_mm, zone_names[tof[0].zone],
                   tof[1].filtered_mm, zone_names[tof[1].zone],
                   tof[2].filtered_mm, zone_names[tof[2].zone],
                   (unsigned long)(loop_us / 1000));
            stdio_flush();
        }

        gpio_put(LED_PIN, 0);
        loop_count++;
        sleep_ms(30);
    }
}