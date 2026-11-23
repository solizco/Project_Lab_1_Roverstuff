/*
 * Rover Phase 1B — Arducam Mega Vision System
 * Board:  Pico 2 (RP2350)
 * Camera: Arducam Mega 3MP (SPI)
 *
 * THREE MODES (compile-time switch):
 *
 *   MODE_STREAM_JPEG    — JPEG to PC visualizer (demo/dev)
 *   MODE_ONDEVICE_RGB   — RGB565 detection on Pico, text results only
 *   MODE_RGB_DEBUG      — RGB565 detection on Pico + streams frame to PC
 *                         so you can SEE what the camera sees while the
 *                         Pico does all the processing
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
    for (uint32_t i = 0; i < len; i++) {
        putchar_raw(data[i]);
    }
}

static void blink_n(int n, uint32_t ms) {
    for (int i = 0; i < n; i++) {
        gpio_put(LED_PIN, 1); sleep_ms(ms);
        gpio_put(LED_PIN, 0); sleep_ms(ms);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  RGB565 DETECTION (shared by MODE_ONDEVICE_RGB and MODE_RGB_DEBUG)
// ═════════════════════════════════════════════════════════════════════════════
#if defined(MODE_ONDEVICE_RGB) || defined(MODE_RGB_DEBUG)

#define FRAME_W 128
#define FRAME_H 128
#define FRAME_PIXELS (FRAME_W * FRAME_H)
#define FRAME_BYTES  (FRAME_PIXELS * 2)

static inline uint8_t rgb565_red(uint8_t hi, uint8_t lo) {
    return (hi & 0xF8);
}
static inline uint8_t rgb565_green(uint8_t hi, uint8_t lo) {
    return ((hi & 0x07) << 5) | ((lo & 0xE0) >> 3);
}
static inline uint8_t rgb565_blue(uint8_t hi, uint8_t lo) {
    return (lo & 0x1F) << 3;
}

static const uint8_t RED_MIN_R = 140;
static const uint8_t RED_MAX_G = 80;
static const uint8_t RED_MAX_B = 80;

struct VisionResult {
    bool     target_found;
    float    target_x;
    float    target_y;
    uint16_t target_size;
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

// Read a full frame from camera FIFO into buffer
static void read_frame_to_buffer(ArducamCamera* cam, uint8_t* frame_buf,
                                  uint32_t total, uint8_t* read_chunk) {
    uint32_t bytesRead = 0;
    while (bytesRead < total && bytesRead < FRAME_BYTES) {
        uint32_t remaining = total - bytesRead;
        uint8_t chunk = (remaining > 200) ? 200 : (uint8_t)remaining;
        readBuff(cam, read_chunk, chunk);
        uint32_t copyLen = chunk;
        if (bytesRead + copyLen > FRAME_BYTES) {
            copyLen = FRAME_BYTES - bytesRead;
        }
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

#endif // MODE_ONDEVICE_RGB || MODE_RGB_DEBUG


// ═════════════════════════════════════════════════════════════════════════════
//  MODE: RGB565 ON-DEVICE ONLY (text output, no frame streaming)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef MODE_ONDEVICE_RGB

int main()
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(500);

    printf("=== RGB565 On-Device Vision ===\n");
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
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
            continue;
        }

        read_frame_to_buffer(cam, frame_buf, total, read_chunk);

        VisionResult vision = process_frame_rgb565(frame_buf);

        if (frame_count % 5 == 0) {
            if (vision.target_found) {
                printf("[FOUND] x=%+.2f y=%+.2f size=%u conf=%.3f\n",
                       vision.target_x, vision.target_y,
                       vision.target_size, vision.confidence);
            } else {
                printf("[NONE]  no red target\n");
            }
            stdio_flush();
        }

        gpio_put(LED_PIN, 0);
        frame_count++;
        sleep_ms(30);
    }
}

#endif // MODE_ONDEVICE_RGB


// ═════════════════════════════════════════════════════════════════════════════
//  MODE: RGB565 DEBUG (on-device detection + stream frame to PC for display)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef MODE_RGB_DEBUG

// Protocol:
//   [0xFF 0xCC]                         — frame+detection packet start
//   [width 1 byte] [height 1 byte]      — frame dimensions
//   [target_found 1 byte]               — 0 or 1
//   [target_x int16 LE]                 — x * 100 as signed int
//   [target_y int16 LE]                 — y * 100 as signed int
//   [target_size uint16 LE]             — red pixel count
//   [capture_us uint16 LE]              — capture time in microseconds
//   [spi_read_us uint16 LE]             — SPI FIFO read time in us
//   [detect_us uint16 LE]               — detection processing time in us
//   [total_us uint16 LE]                — total loop time in us
//   [cpu_free_pct uint8]                — estimated CPU free percentage
//   [RGB565 frame data]                 — width*height*2 bytes
//   [0xFF 0xDD]                         — packet end

int main()
{
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
    

    // Target loop period in microseconds (how fast we WANT to run)
    // 30ms sleep + processing = ~50ms target = 20 FPS goal
    const uint32_t TARGET_LOOP_US = 50000;

    while (true) {
        gpio_put(LED_PIN, 1);
        uint32_t loop_start = time_us_32();

        // ── CAPTURE: tell camera to take a picture ───────────────────────
        uint32_t t0 = time_us_32();
        myCAM.takePicture((CAM_IMAGE_MODE)CAM_IMAGE_MODE_128X128,
                          (CAM_IMAGE_PIX_FMT)CAM_IMAGE_PIX_FMT_RGB565);
        uint32_t capture_us = time_us_32() - t0;

        uint32_t total = myCAM.getTotalLength();
        if (total == 0 || total > FRAME_BYTES * 2) {
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
            continue;
        }

        // ── SPI READ: pull frame from camera FIFO ────────────────────────
        t0 = time_us_32();
        read_frame_to_buffer(cam, frame_buf, total, read_chunk);
        uint32_t spi_read_us = time_us_32() - t0;

        // ── DETECT: process pixels for red ───────────────────────────────
        t0 = time_us_32();
        VisionResult vision = process_frame_rgb565(frame_buf);
        uint32_t detect_us = time_us_32() - t0;

        // ── USB SEND: stream results + frame to PC ───────────────────────
        t0 = time_us_32();

        uint8_t header[2] = {0xFF, 0xCC};
        send_raw_bytes(header, 2);

        // Dimensions
        putchar_raw((uint8_t)FRAME_W);
        putchar_raw((uint8_t)FRAME_H);

        // Detection results
        putchar_raw(vision.target_found ? 1 : 0);

        int16_t tx = (int16_t)(vision.target_x * 100);
        putchar_raw((uint8_t)(tx & 0xFF));
        putchar_raw((uint8_t)((tx >> 8) & 0xFF));

        int16_t ty = (int16_t)(vision.target_y * 100);
        putchar_raw((uint8_t)(ty & 0xFF));
        putchar_raw((uint8_t)((ty >> 8) & 0xFF));

        putchar_raw((uint8_t)(vision.target_size & 0xFF));
        putchar_raw((uint8_t)((vision.target_size >> 8) & 0xFF));

        // Performance data
        uint16_t cap_us16  = (capture_us > 65535)  ? 65535 : (uint16_t)capture_us;
        uint16_t spi_us16  = (spi_read_us > 65535) ? 65535 : (uint16_t)spi_read_us;
        uint16_t det_us16  = (detect_us > 65535)   ? 65535 : (uint16_t)detect_us;

        uint32_t total_us = time_us_32() - loop_start;
        uint16_t tot_us16  = (total_us > 65535)    ? 65535 : (uint16_t)total_us;

        // CPU free = how much of our target loop time is unused
        uint8_t cpu_free = 0;
        if (total_us < TARGET_LOOP_US) {
            cpu_free = (uint8_t)(((TARGET_LOOP_US - total_us) * 100) / TARGET_LOOP_US);
        }

        putchar_raw((uint8_t)(cap_us16 & 0xFF));
        putchar_raw((uint8_t)((cap_us16 >> 8) & 0xFF));
        putchar_raw((uint8_t)(spi_us16 & 0xFF));
        putchar_raw((uint8_t)((spi_us16 >> 8) & 0xFF));
        putchar_raw((uint8_t)(det_us16 & 0xFF));
        putchar_raw((uint8_t)((det_us16 >> 8) & 0xFF));
        putchar_raw((uint8_t)(tot_us16 & 0xFF));
        putchar_raw((uint8_t)((tot_us16 >> 8) & 0xFF));
        putchar_raw(cpu_free);

        // Raw RGB565 frame
        send_raw_bytes(frame_buf, FRAME_BYTES);

        // Footer
        uint8_t footer[2] = {0xFF, 0xDD};
        send_raw_bytes(footer, 2);
        stdio_flush();

        gpio_put(LED_PIN, 0);
        sleep_ms(30);
    }
}

#endif // MODE_RGB_DEBUG


// ═════════════════════════════════════════════════════════════════════════════
//  MODE: JPEG STREAMING (for PC OpenCV visualizer)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef MODE_STREAM_JPEG

int main()
{
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
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
            continue;
        }

        uint8_t header[6] = {
            0xFF, 0xAA,
            (uint8_t)(total & 0xFF),
            (uint8_t)((total >> 8) & 0xFF),
            (uint8_t)((total >> 16) & 0xFF),
            (uint8_t)((total >> 24) & 0xFF)
        };
        send_raw_bytes(header, 6);

        uint32_t bytesRemaining = total;
        while (bytesRemaining > 0) {
            uint8_t chunk = (bytesRemaining > 200) ? 200 : (uint8_t)bytesRemaining;
            readBuff(cam, readBuf, chunk);
            send_raw_bytes(readBuf, chunk);
            bytesRemaining -= chunk;
        }

        uint8_t footer[2] = {0xFF, 0xBB};
        send_raw_bytes(footer, 2);
        stdio_flush();

        gpio_put(LED_PIN, 0);
        sleep_ms(30);
    }
}

#endif // MODE_STREAM_JPEG