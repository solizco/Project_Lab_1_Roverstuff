#pragma once
// =============================================================================
// field_boundary.h
//
// Coordinate system (encoder-relative; origin = where the rover is placed at INIT):
//   enc.x() = forward, toward the opponent (12 ft / 3658 mm field axis)
//   enc.y() = lateral  (7 ft / 2134 mm field axis)
//   theta=0 = facing the opponent goal
//
// Two switches are read at boot (see config_pins.h):
//   Role   (mode_switch, GP14):               open=HIGH=KICKER  closed=LOW=GOALIE
//   Corner (corner_orientation_switch, GP9):  open=HIGH=LEFT    closed=LOW=RIGHT
//
// The role changes the forward brake limit (a goalie stays in its own half).
// The corner is reported at INIT; origin_x()/origin_y()/lateral_heading()
// describe a corner-aware start position but the competition firmware
// returns to the placement point (0, 0) instead.
//
// Brake/warn limits are relative to the placement point and symmetric
// laterally (brake at ±1750 mm), so they do not depend on the starting corner.
// Near a side wall the rover relies on its ToF sensors rather than these limits.
// =============================================================================

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "config_pins.h"
#include <cmath>
#include <cstdint>

static constexpr uint PIN_ROLE_SW   = mode_switch;
static constexpr uint PIN_CORNER_SW = corner_orientation_switch;

// ── Field dimensions (mm) ────────────────────────────────────────────────────
static constexpr float FIELD_L_MM = 3658.0f;   // 12ft — forward X axis
static constexpr float FIELD_W_MM = 2134.0f;   // 7ft  — lateral Y axis
static constexpr float HALF_L_MM  = FIELD_L_MM / 2.0f;  // 1829mm centerline

// ── Goal geometry ─────────────────────────────────────────────────────────────
static constexpr float GOAL_W_MM  = 914.0f;    // 3ft wide
static constexpr float GOAL_D_MM  = 305.0f;    // 1ft deep

// ── Origin (fire/return position) ─────────────────────────────────────────────
// X: 6in (152mm) clearance past goal front face = 305 + 152 = 457mm
// Y: 1/3 of goal width from the near wall
static constexpr float ORIGIN_X_MM       = GOAL_D_MM + 152.0f;  // 457mm
static constexpr float ORIGIN_Y_NEAR     = GOAL_W_MM / 3.0f;    // 304mm from near wall
static constexpr float ORIGIN_Y_FAR      = FIELD_W_MM - ORIGIN_Y_NEAR; // 1830mm from Y=0

// ── Safety margins ────────────────────────────────────────────────────────────
static constexpr float MARGIN_MM  = 50.0f;   // hard brake from wall
static constexpr float WARN_MM    = 75.0f;   // soft warn from wall

// =============================================================================
enum class BoundaryAction : uint8_t { SAFE = 0, WARN = 1, BRAKE = 2 };

struct BoundaryCmd {
    float left;
    float right;
    bool  active;
};

enum class RoverRole   : uint8_t { KICKER = 0, GOALIE = 1 };
enum class RoverCorner : uint8_t { LEFT   = 0, RIGHT  = 1 };

// =============================================================================
class FieldBoundary {
public:
    FieldBoundary() = default;

    void init() {
        gpio_init(PIN_ROLE_SW);
        gpio_set_dir(PIN_ROLE_SW, GPIO_IN);
        gpio_pull_up(PIN_ROLE_SW);

        gpio_init(PIN_CORNER_SW);
        gpio_set_dir(PIN_CORNER_SW, GPIO_IN);
        gpio_pull_up(PIN_CORNER_SW);
        sleep_ms(10);

        role_   = gpio_get(PIN_ROLE_SW)   ? RoverRole::KICKER  : RoverRole::GOALIE;
        corner_ = gpio_get(PIN_CORNER_SW) ? RoverCorner::LEFT : RoverCorner::RIGHT;

        compute_limits();
        reset_escape_state();
    }

    void reset_escape_state() {
        escaping_         = false;
        escape_ticks_rem_ = 0;
    }

    // ── Origin accessors ──────────────────────────────────────────────────────
    // X is always the same — 457mm forward from own wall.
    // Y depends on corner: RIGHT corner is near Y=0, LEFT is near Y=2134.
    float origin_x() const { return ORIGIN_X_MM; }
    float origin_y() const {
        return is_right_corner() ? ORIGIN_Y_NEAR   // 304mm from Y=0 wall
                                 : ORIGIN_Y_FAR;   // 1830mm from Y=0 wall
    }

    // ── Lateral drive direction for INIT ──────────────────────────────────────
    // RIGHT corner: against Y=0 wall, drive +Y toward goal
    // LEFT corner:  against Y=2134 wall, drive -Y toward goal
    float lateral_heading() const {
        return is_right_corner() ?  (float)M_PI / 2.0f   // +Y
                                 : -(float)M_PI / 2.0f;  // -Y
    }
    float lateral_dist_mm() const { return ORIGIN_Y_NEAR; }  // 304mm either direction

    // ── Role / corner queries ─────────────────────────────────────────────────
    bool is_kicker()      const { return role_   == RoverRole::KICKER;  }
    bool is_goalie()      const { return role_   == RoverRole::GOALIE;  }
    bool is_left_corner() const { return corner_ == RoverCorner::LEFT;  }
    bool is_right_corner()const { return corner_ == RoverCorner::RIGHT; }

    // ── Boundary check ────────────────────────────────────────────────────────
    BoundaryAction check(float x, float y) const {
        if (in_brake_zone(x, y)) return BoundaryAction::BRAKE;
        if (in_warn_zone (x, y)) return BoundaryAction::WARN;
        return BoundaryAction::SAFE;
    }

    BoundaryCmd correction_command(float x, float y, float theta) {
    BoundaryCmd cmd{};

    // Identify which wall(s) are being violated

    bool hit_y_lo = (y < min_y_);   // lateral wall near Y=0
    bool hit_y_hi = (y > max_y_);   // lateral wall near Y=2134

    if (in_brake_zone(x, y)) {
        if (!escaping_) {
            escaping_         = true;
            escape_ticks_rem_ = ESCAPE_TICKS;
        }

        if (escape_ticks_rem_ > 0) {
            --escape_ticks_rem_;

            if (hit_y_lo || hit_y_hi) {
                // ── Lateral wall hit — turn to face field center ──────────────
                // Target heading points toward center Y (FIELD_W_MM/2) from
                // current position. This is a 180-style pivot away from wall.
                float center_y    = FIELD_W_MM / 2.0f;
                float desired     = atan2f(center_y - y, 50.0f);  // always drive toward X+
                float err         = desired - theta;
                while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
                while (err < -(float)M_PI) err += 2.0f * (float)M_PI;
                float turn = clampf(err / (float)M_PI, -1.0f, 1.0f);

                // Spin to face center — forward drive + strong turn
                cmd.left   =  0.5f + turn * 0.5f;
                cmd.right  =  0.5f - turn * 0.5f;
                cmd.active = true;

            } else {
                // ── X wall hit (own goal or opponent/centerline) — reverse straight
                cmd.left   = -0.6f;
                cmd.right  = -0.6f;
                cmd.active = true;
            }

        } else {
            escaping_         = false;
            escape_ticks_rem_ = 0;
            cmd.left = cmd.right = 0.0f;
            cmd.active = true;
        }

    } else {
    escaping_         = false;
    escape_ticks_rem_ = 0;
    // Warn zone — nudge toward safe area without committing to a direction
    // turn_toward_safe already accounts for which wall we're near
    float turn = turn_toward_safe(x, y, theta) * 0.5f;
    // Spin toward the safe area; once roughly facing it, creep forward so the
    // rover actually leaves the warn zone instead of stopping inside it.
    float spd  = (fabsf(turn) < WARN_FACING_TOL) ? WARN_CREEP_SPEED : 0.0f;
    cmd.left   = spd + turn * 0.82f;
    cmd.right  = spd - turn * 0.82f;
    cmd.active = true;
}

    return cmd;
}

    bool is_escaping() const { return escaping_; }

    void print_config() const {
        printf("[Boundary] Role:   %s\n", is_goalie() ? "GOALIE" : "KICKER");
        printf("[Boundary] Corner: %s\n", is_left_corner() ? "LEFT" : "RIGHT");
        printf("[Boundary] Origin: (%.0f, %.0f) mm\n", origin_x(), origin_y());
        printf("[Boundary] X brake: %.0f .. %.0f mm\n", min_x_, max_x_);
        printf("[Boundary] Y brake: %.0f .. %.0f mm\n", min_y_, max_y_);
    }

private:
    RoverRole   role_   = RoverRole::KICKER;
    RoverCorner corner_ = RoverCorner::LEFT;

    float min_x_     = -400.0f;
    float max_x_     =  3200.0f;
    float min_y_     = -1800.0f;
    float max_y_     =  1800.0f;
    float warn_x_lo_ = -300.0f;
    float warn_x_hi_ =  3100.0f;
    float warn_y_lo_ = -1700.0f;
    float warn_y_hi_ =  1700.0f;

    void compute_limits() {
        min_x_     = -400.0f;
        warn_x_lo_ = -300.0f;

        if (role_ == RoverRole::GOALIE) {
            max_x_     = 1372.0f - MARGIN_MM;
            warn_x_hi_ = 1372.0f - WARN_MM;
        } else {
            max_x_     = 3200.0f - MARGIN_MM;
            warn_x_hi_ = 3200.0f - WARN_MM;
        }

        min_y_     = -1800.0f + MARGIN_MM;
        max_y_     =  1800.0f - MARGIN_MM;
        warn_y_lo_ = -1800.0f + WARN_MM;
        warn_y_hi_ =  1800.0f - WARN_MM;
    }

    bool in_brake_zone(float x, float y) const {
        return (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_);
    }

    bool in_warn_zone(float x, float y) const {
        return (x < warn_x_lo_ || x > warn_x_hi_ ||
                y < warn_y_lo_ || y > warn_y_hi_);
    }

    float turn_toward_safe(float x, float y, float theta) const {
        float tx = clampf(x, warn_x_lo_ + 50.f, warn_x_hi_ - 50.f);
        float ty = clampf(y, warn_y_lo_ + 50.f, warn_y_hi_ - 50.f);
        float desired = atan2f(ty - y, tx - x);
        float err = desired - theta;
        while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
        while (err < -(float)M_PI) err += 2.0f * (float)M_PI;
        return clampf(err / (float)M_PI, -1.0f, 1.0f);
    }

    static float clampf(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    static constexpr int   ESCAPE_TICKS     = 80;
    static constexpr float WARN_FACING_TOL  = 0.08f;  // |turn| below this ≈ within ~30° of safe heading
    static constexpr float WARN_CREEP_SPEED = 0.72f;
    bool escaping_         = false;
    int  escape_ticks_rem_ = 0;
};