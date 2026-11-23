/*
 * Rover Phase 1B — Arducam Mega Vision System
 * Board:  Pico 2 (RP2350)
 * Camera: Arducam Mega 3MP (SPI)
 *
 * THREE MODES (compile-time switch):
 *   MODE_STREAM_JPEG    — JPEG to PC visualizer
 *   MODE_ONDEVICE_RGB   — Detection, text output only (serial monitor)
 *   MODE_RGB_DEBUG      — Detection + stream frame to PC visualizer
 *
 * Detection: grid density + two-pass cluster centroid.
 *   Pass 1: 16x16 grid of 8x8 cells. Densest cell = closest ball.
 *            Works with multiple balls — each ball lands in its own cell.
 *   Pass 2: tight cluster within CLUSTER_RADIUS=20 of densest cell center.
 *            Small radius ensures only ONE ball's pixels are collected,
 *            preventing two nearby balls from merging into one huge bbox.
 *
 * Color detection: channel difference + absolute ceiling.
 *   Red: diff > 60 AND g < 80 AND b < 80 — rejects skin (g=120) and brown.
 *   Green/Blue: diff > 35 — works at field distance under ECE 101 lighting.
 *
 * MODE_RGB_DEBUG packet format:
 *   [0xFF 0xCC]           start
 *   [w][h]                frame dims
 *   [found]               0 or 1
 *   [tx(2LE)]             target_x * 100 signed int16
 *   [ty(2LE)]             target_y * 100 signed int16
 *   [size(2LE)]           colored pixel count uint16
 *   [bx0][by0][bx1][by1]  bounding box
 *   [circ]                circularity 0-100
 *   [fill]                fill ratio 0-100
 *   [cap(2LE)]            capture us
 *   [spi(2LE)]            spi read us
 *   [det(2LE)]            detect us
 *   [tot(2LE)]            total loop us
 *   [cpu]                 cpu free %
 *   [RGB565 frame]        w*h*2 bytes
 *   [0xFF 0xDD]           end
 *
 * Header after magic: 26 bytes
 */

// Uncomment ONE:
// #define MODE_STREAM_JPEG
// #define MODE_ONDEVICE_RGB
#define MODE_RGB_DEBUG

#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "Arducam_Mega.h"
#include "ArducamCamera.h"

static const int CS_PIN  = 17;
static const int LED_PIN = 25;

Arducam_Mega myCAM(CS_PIN);

static void send_raw_bytes(const uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) putchar_raw(data[i]);
}

static void blink_n(int n, uint32_t ms) {
    for (int i = 0; i < n; i++) {
        gpio_put(LED_PIN, 1); sleep_ms(ms);
        gpio_put(LED_PIN, 0); sleep_ms(ms);
    }
}

// =========================================================================
//  DETECTION — shared by all non-JPEG modes
// =========================================================================
#if defined(MODE_ONDEVICE_RGB) || defined(MODE_RGB_DEBUG)

#define FRAME_W      128
#define FRAME_H      128
#define FRAME_PIXELS (FRAME_W * FRAME_H)
#define FRAME_BYTES  (FRAME_PIXELS * 2)

// ── Detection filters ─────────────────────────────────────────────────────────
#define MIN_COLOR_PIX      2     // distant ball at 6ft may only be 2-4 pixels
#define MAX_COLOR_PIX  12000     // close-up ball can fill most of 128x128
#define MAX_ASPECT       1.6f    // bbox w/h — balls are round
#define MIN_FILL         0.45f   // colored pix / bbox area
// CLUSTER_RADIUS=20: small enough that two balls 50px apart don't merge.
// A single close-up ball fits within 20px of its grid cell center.
#define CLUSTER_RADIUS   20.0f
#define MIN_BBOX_DIM      2      // allow 2x2px bbox for distant balls

struct VisionResult {
    bool     target_found;
    float    target_x;
    float    target_y;
    uint16_t target_size;
    float    confidence;
    float    fill_ratio;
    int16_t  bbox_x0, bbox_y0, bbox_x1, bbox_y1;
};

// ── Per-pixel color test ──────────────────────────────────────────────────────
// Red:   diff > 60 AND g < 80 AND b < 80
//        Skin: R=180 G=120 B=100 → g=120 FAILS g<80 ✓
//        Red ball: R=216 G=48 B=48 → diff=168 g=48 PASSES ✓
// Green: diff > 35 — floor tile and walls are neutral, won't pass
// Blue:  diff > 35 / > 25 — same
static inline bool rgb565_is_ball_color(uint8_t hi, uint8_t lo) {
    uint8_t r = (hi & 0xF8);
    uint8_t g = ((hi & 0x07) << 5) | ((lo & 0xE0) >> 3);
    uint8_t b = (lo & 0x1F) << 3;

    uint8_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    if (mx < 40) return false;

    // Red: strict — rejects skin and brown clothing
    if (r >= 90 && (int)r - (int)g > 60 && (int)r - (int)b > 60
        && g < 80 && b < 80) return true;

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

    // Find densest cell — closest/largest ball wins
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

    if (best_count < 2)  return result;
    if (best_count > 55) return result;                     // wall/shirt fills whole cell
    if (best_gy < (GROWS * 40 / 100)) return result;       // top 40% = background
    // Seed = center of densest cell
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

    if (asp  > MAX_ASPECT) return result;
    if (fill < MIN_FILL)   return result;

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

static void read_frame_to_buffer(ArducamCamera* cam, uint8_t* frame_buf,
                                  uint32_t total, uint8_t* read_chunk) {
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

#endif // MODE_ONDEVICE_RGB || MODE_RGB_DEBUG


// =========================================================================
//  MODE: TEXT ONLY (serial monitor)
// =========================================================================
#ifdef MODE_ONDEVICE_RGB

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(500);

    printf("=== Ball Detection (grid density + cluster) ===\n");
    printf("MIN_PIX=%d  MAX_PIX=%d  MIN_FILL=%.2f  RADIUS=%.0f  MIN_DIM=%d\n",
           MIN_COLOR_PIX, MAX_COLOR_PIX, MIN_FILL, CLUSTER_RADIUS, MIN_BBOX_DIM);
    stdio_flush();

    blink_n(2, 200);
    myCAM.begin();
    blink_n(3, 200);

    ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t frame_buf[FRAME_BYTES];
    static uint8_t read_chunk[200];
    blink_n(5, 100);

    uint32_t frame_count = 0;

    while (true) {
        gpio_put(LED_PIN, 1);
        myCAM.takePicture((CAM_IMAGE_MODE)CAM_IMAGE_MODE_128X128,
                          (CAM_IMAGE_PIX_FMT)CAM_IMAGE_PIX_FMT_RGB565);
        uint32_t total = myCAM.getTotalLength();
        if (total == 0 || total > FRAME_BYTES * 2) {
            gpio_put(LED_PIN, 0); sleep_ms(100); continue;
        }
        read_frame_to_buffer(cam, frame_buf, total, read_chunk);
        VisionResult v = process_frame_blob(frame_buf);

        if (frame_count % 5 == 0) {
            if (v.target_found)
                printf("[BALL] x=%+.2f y=%+.2f pix=%u circ=%.2f fill=%.2f "
                       "bbox=[%d,%d-%d,%d]\n",
                       v.target_x, v.target_y, v.target_size,
                       v.confidence, v.fill_ratio,
                       v.bbox_x0, v.bbox_y0, v.bbox_x1, v.bbox_y1);
            else
                printf("[NONE] no ball\n");
            stdio_flush();
        }

        gpio_put(LED_PIN, 0);
        frame_count++;
        sleep_ms(30);
    }
}

#endif // MODE_ONDEVICE_RGB


// =========================================================================
//  MODE: BLOB DEBUG (stream frame + detection to PC visualizer)
// =========================================================================
#ifdef MODE_RGB_DEBUG

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(500);

    blink_n(2, 200);
    myCAM.begin();
    blink_n(3, 200);

    ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t frame_buf[FRAME_BYTES];
    static uint8_t read_chunk[200];
    blink_n(5, 100);

    const uint32_t TARGET_LOOP_US = 50000;

    while (true) {
        gpio_put(LED_PIN, 1);
        uint32_t loop_start = time_us_32();

        uint32_t t0 = time_us_32();
        myCAM.takePicture((CAM_IMAGE_MODE)CAM_IMAGE_MODE_128X128,
                          (CAM_IMAGE_PIX_FMT)CAM_IMAGE_PIX_FMT_RGB565);
        uint32_t capture_us = time_us_32() - t0;

        uint32_t total = myCAM.getTotalLength();
        if (total == 0 || total > FRAME_BYTES * 2) {
            gpio_put(LED_PIN, 0); sleep_ms(100); continue;
        }

        t0 = time_us_32();
        read_frame_to_buffer(cam, frame_buf, total, read_chunk);
        uint32_t spi_read_us = time_us_32() - t0;

        t0 = time_us_32();
        VisionResult v = process_frame_blob(frame_buf);
        uint32_t detect_us = time_us_32() - t0;

        uint8_t hdr[2] = {0xFF, 0xCC};
        send_raw_bytes(hdr, 2);

        putchar_raw((uint8_t)FRAME_W);
        putchar_raw((uint8_t)FRAME_H);
        putchar_raw(v.target_found ? 1 : 0);

        int16_t tx = (int16_t)(v.target_x * 100);
        putchar_raw((uint8_t)(tx & 0xFF));
        putchar_raw((uint8_t)((tx >> 8) & 0xFF));

        int16_t ty = (int16_t)(v.target_y * 100);
        putchar_raw((uint8_t)(ty & 0xFF));
        putchar_raw((uint8_t)((ty >> 8) & 0xFF));

        putchar_raw((uint8_t)(v.target_size & 0xFF));
        putchar_raw((uint8_t)((v.target_size >> 8) & 0xFF));

        putchar_raw((uint8_t)v.bbox_x0);
        putchar_raw((uint8_t)v.bbox_y0);
        putchar_raw((uint8_t)v.bbox_x1);
        putchar_raw((uint8_t)v.bbox_y1);
        putchar_raw((uint8_t)(v.confidence * 100.0f));
        putchar_raw((uint8_t)(v.fill_ratio  * 100.0f));

        uint32_t total_us = time_us_32() - loop_start;
        uint16_t cap_us16 = (capture_us  > 65535) ? 65535 : (uint16_t)capture_us;
        uint16_t spi_us16 = (spi_read_us > 65535) ? 65535 : (uint16_t)spi_read_us;
        uint16_t det_us16 = (detect_us   > 65535) ? 65535 : (uint16_t)detect_us;
        uint16_t tot_us16 = (total_us    > 65535) ? 65535 : (uint16_t)total_us;
        uint8_t  cpu_free = (total_us < TARGET_LOOP_US)
                            ? (uint8_t)(((TARGET_LOOP_US - total_us) * 100) / TARGET_LOOP_US)
                            : 0;

        putchar_raw((uint8_t)(cap_us16 & 0xFF)); putchar_raw((uint8_t)(cap_us16 >> 8));
        putchar_raw((uint8_t)(spi_us16 & 0xFF)); putchar_raw((uint8_t)(spi_us16 >> 8));
        putchar_raw((uint8_t)(det_us16 & 0xFF)); putchar_raw((uint8_t)(det_us16 >> 8));
        putchar_raw((uint8_t)(tot_us16 & 0xFF)); putchar_raw((uint8_t)(tot_us16 >> 8));
        putchar_raw(cpu_free);

        send_raw_bytes(frame_buf, FRAME_BYTES);

        uint8_t footer[2] = {0xFF, 0xDD};
        send_raw_bytes(footer, 2);
        stdio_flush();

        gpio_put(LED_PIN, 0);
        sleep_ms(30);
    }
}

#endif // MODE_RGB_DEBUG


// =========================================================================
//  MODE: JPEG STREAMING (unchanged)
// =========================================================================
#ifdef MODE_STREAM_JPEG

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(500);

    blink_n(2, 200);
    myCAM.begin();
    blink_n(3, 200);

    ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t readBuf[200];
    blink_n(5, 100);

    while (true) {
        gpio_put(LED_PIN, 1);
        myCAM.takePicture((CAM_IMAGE_MODE)CAM_IMAGE_MODE_QVGA,
                          (CAM_IMAGE_PIX_FMT)CAM_IMAGE_PIX_FMT_JPG);
        uint32_t total = myCAM.getTotalLength();
        if (total == 0 || total > 100000) {
            gpio_put(LED_PIN, 0); sleep_ms(100); continue;
        }

        uint8_t hdr[6] = {
            0xFF, 0xAA,
            (uint8_t)(total & 0xFF),
            (uint8_t)((total >> 8)  & 0xFF),
            (uint8_t)((total >> 16) & 0xFF),
            (uint8_t)((total >> 24) & 0xFF)
        };
        send_raw_bytes(hdr, 6);

        uint32_t rem = total;
        while (rem > 0) {
            uint8_t chunk = (rem > 200) ? 200 : (uint8_t)rem;
            readBuff(cam, readBuf, chunk);
            send_raw_bytes(readBuf, chunk);
            rem -= chunk;
        }

        uint8_t footer[2] = {0xFF, 0xBB};
        send_raw_bytes(footer, 2);
        stdio_flush();

        gpio_put(LED_PIN, 0);
        sleep_ms(30);
    }
}

#endif // MODE_STREAM_JPEG