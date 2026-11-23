#pragma once
// =============================================================================
// wheel_encoder.h
// Translated from Finn's nav_test.py (MicroPython rev 2) to C++ / Pico SDK.
//
// Changes from rev 1:
//   - WHEEL_DIAM_MM  25.7  → 63.5  (measured)
//   - WHEELBASE_MM   169.5 → 177.0 (measured)
//   - update_odometry() now disables IRQs around tick snapshot (mirrors
//     Finn's machine.disable_irq() / machine.enable_irq() pattern)
//
// Encoder GPIO pins (match Finn's hardware):
//   ENC_LEFT_PIN  = GP2
//   ENC_RIGHT_PIN = GP3
// =============================================================================

#include "pico/stdlib.h"
#include <cmath>
#include <cstdint>
#include "hardware/sync.h"
#include "config_pins.h"

// ── Physical constants ────────────────────────────────────────────────────────
static constexpr int   TICKS_PER_REV = 20;
static constexpr float WHEEL_DIAM_MM = 63.5f;
static constexpr float WHEELBASE_MM  = 177.0f;
static constexpr float MM_PER_TICK   = (float)(M_PI * WHEEL_DIAM_MM / TICKS_PER_REV);

// ── Debounce ──────────────────────────────────────────────────────────────────
static constexpr uint32_t DEBOUNCE_US = 1000;

// ── Encoder GPIO pins ─────────────────────────────────────────────────────────
static constexpr uint ENC_LEFT_PIN  = ENC_PIN_LEFT;
static constexpr uint ENC_RIGHT_PIN = ENC_PIN_RIGHT;

// =============================================================================
// WheelEncoder
// =============================================================================
class WheelEncoder {
public:
    WheelEncoder() {}

    // Call once at startup — configures GPIOs and attaches IRQs.
    void init();

    // ── Direction control ─────────────────────────────────────────────────────
    // Call whenever motor direction changes so tick sign stays correct.
    // +1 forward, -1 reverse, 0 stopped — mirrors Finn's left_dir / right_dir.
    void set_left_dir(int d)  { left_dir_  = d; }
    void set_right_dir(int d) { right_dir_ = d; }

    // ── IRQ-safe tick reads ───────────────────────────────────────────────────
    // Disables interrupts for the snapshot, same as Finn's machine.disable_irq()
    int32_t left_ticks() const {
        uint32_t s = save_and_disable_interrupts();
        int32_t  v = enc_left_ticks_;
        restore_interrupts(s);
        return v;
    }
    int32_t right_ticks() const {
        uint32_t s = save_and_disable_interrupts();
        int32_t  v = enc_right_ticks_;
        restore_interrupts(s);
        return v;
    }

    void reset_ticks() {
        uint32_t s = save_and_disable_interrupts();
        enc_left_ticks_  = 0;
        enc_right_ticks_ = 0;
        restore_interrupts(s);
        last_left_odo_  = 0;
        last_right_odo_ = 0;
    }

    // ── Odometry ──────────────────────────────────────────────────────────────
    // Call from main loop (never from ISR).
    // Takes an IRQ-safe snapshot of ticks then computes pose — matches Finn's
    // machine.disable_irq() pattern in update_odometry().
    void update_odometry() {
        // Atomic snapshot — mirrors Finn's machine.disable_irq() block
        uint32_t s = save_and_disable_interrupts();
        int32_t  cur_left  = enc_left_ticks_;
        int32_t  cur_right = enc_right_ticks_;
        restore_interrupts(s);

        float d_left  = (cur_left  - last_left_odo_)  * MM_PER_TICK;
        float d_right = (cur_right - last_right_odo_) * MM_PER_TICK;
        last_left_odo_  = cur_left;
        last_right_odo_ = cur_right;

        float d_center = (d_left + d_right) / 2.0f;
        float dtheta   = (d_right - d_left) / WHEELBASE_MM;

        x_     += d_center * cosf(theta_ + dtheta / 2.0f);
        y_     += d_center * sinf(theta_ + dtheta / 2.0f);
        theta_ += dtheta;
        // Wrap theta to [-π, π] to prevent unbounded accumulation
        while (theta_ >  (float)M_PI) theta_ -= 2.0f * (float)M_PI;
        while (theta_ < -(float)M_PI) theta_ += 2.0f * (float)M_PI;
    }

    // ── Pose ──────────────────────────────────────────────────────────────────
    float x()     const { return x_; }
    float y()     const { return y_; }
    float theta() const { return theta_; }

    void reset_pose(float x = 0.f, float y = 0.f, float t = 0.f) {
        x_ = x; y_ = y; theta_ = t;
        // Re-anchor odometry to current tick counts so deltas start at zero
        last_left_odo_  = left_ticks();
        last_right_odo_ = right_ticks();
    }

    // ── Utility ───────────────────────────────────────────────────────────────
    float distance_to_origin() const { return sqrtf(x_*x_ + y_*y_); }
    float angle_to_origin()    const { return atan2f(-y_, -x_); }

    static float normalize_angle(float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    }

    // ── ISR callbacks — called from gpio IRQ shim, do not call directly ───────
    void _isr_left(uint /*gpio*/, uint32_t /*events*/) {
        uint32_t now = time_us_32();
        if ((now - last_left_time_) > DEBOUNCE_US) {
            enc_left_ticks_ += left_dir_;
            last_left_time_  = now;
        }
    }
    void _isr_right(uint /*gpio*/, uint32_t /*events*/) {
        uint32_t now = time_us_32();
        if ((now - last_right_time_) > DEBOUNCE_US) {
            enc_right_ticks_ += right_dir_;
            last_right_time_  = now;
        }
    }

private:
    volatile int32_t  enc_left_ticks_  = 0;
    volatile int32_t  enc_right_ticks_ = 0;
    volatile uint32_t last_left_time_  = 0;
    volatile uint32_t last_right_time_ = 0;
    volatile int      left_dir_        = 0;
    volatile int      right_dir_       = 0;

    float   x_             = 0.f;
    float   y_             = 0.f;
    float   theta_         = 0.f;
    int32_t last_left_odo_  = 0;
    int32_t last_right_odo_ = 0;
};

extern WheelEncoder* g_encoder_instance;