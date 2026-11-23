// =============================================================================
// encoder_test.cpp
//
// Full navigation test — direct C++ translation of Finn's nav_test.py rev 2.
//
// All pin assignments pulled from config_pins.h — no hardcoded GP numbers.
//
// Motor pins (from config_pins.h):
//   MOTOR_L_PIN_IN3 = GP26  (left  forward PWM, slice5 chanA)
//   MOTOR_L_PIN_IN4 = GP27  (left  reverse PWM, slice5 chanB)
//   MOTOR_R_PIN_IN1 = GP22  (right forward PWM, slice3 chanA)
//   MOTOR_R_PIN_IN2 = GP28  (right reverse PWM, slice6 chanA)
//
// Encoder pins (from config_pins.h):
//   ENC_PIN_LEFT  = GP2
//   ENC_PIN_RIGHT = GP3   (no conflict — GP3 is free in current config_pins.h)
//
// Build target — add to CMakeLists.txt:
//   add_executable(encoder_test encoder_test.cpp wheel_encoder.cpp)
//   target_link_libraries(encoder_test pico_stdlib hardware_gpio hardware_pwm)
//   pico_enable_stdio_usb(encoder_test 1)
//   pico_enable_stdio_uart(encoder_test 0)
//   pico_add_extra_outputs(encoder_test)
// =============================================================================

#include "config_pins.h"
#include "wheel_encoder.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// ── PWM setup ─────────────────────────────────────────────────────────────────
// Match Finn's 16-bit duty scale (0-65535).
// RP2350 sys_clk = 150 MHz → clkdiv = 150e6 / (65535 * 1000) ≈ 2.29 → ~1kHz
#define PWM_WRAP  65535u

static void pwm_pin_init(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, 2.29f);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
}

static void pwm_duty(uint pin, uint32_t duty) {
    if (duty > PWM_WRAP) duty = PWM_WRAP;
    pwm_set_gpio_level(pin, (uint16_t)duty);
}

// ── Motor speeds ──────────────────────────────────────────────────────────────
#define DRIVE_SPEED      35000u
#define TURN_SPEED       30000u
#define STRAIGHT_BIAS    1400u
#define LEFT_TURN_BIAS   400u
#define RIGHT_TURN_BIAS  0u

// ── Navigation tolerances ─────────────────────────────────────────────────────
#define ANGLE_TOL   0.06f    // radians (~3.4 deg)
#define DIST_TOL    15.0f    // mm
#define TRIM_KP     400      // proportional tick-error gain

// ── Control loop interval ─────────────────────────────────────────────────────
#define LOOP_MS  20

// ── Encoder instance ──────────────────────────────────────────────────────────
static WheelEncoder enc;

// =============================================================================
// Motor primitives — all pins from config_pins.h
// =============================================================================

static void _set_left(uint32_t fwd, uint32_t rev) {
    pwm_duty(MOTOR_L_PIN_IN3, fwd);
    pwm_duty(MOTOR_L_PIN_IN4, rev);
}

static void _set_right(uint32_t fwd, uint32_t rev) {
    pwm_duty(MOTOR_R_PIN_IN1, fwd);
    pwm_duty(MOTOR_R_PIN_IN2, rev);
}

static void motor_stop() {
    _set_left(0, 0);
    _set_right(0, 0);
    enc.set_left_dir(0);
    enc.set_right_dir(0);
}

static void motor_forward(uint32_t l_speed = DRIVE_SPEED,
                          uint32_t r_speed = DRIVE_SPEED) {
    uint32_t lf = (l_speed > STRAIGHT_BIAS) ? l_speed - STRAIGHT_BIAS : 0;
    uint32_t rf = r_speed + STRAIGHT_BIAS;
    if (rf > PWM_WRAP) rf = PWM_WRAP;
    _set_left(lf, 0);
    _set_right(rf, 0);
    enc.set_left_dir(1);
    enc.set_right_dir(1);
}

// Spin left: left wheel reverse, right wheel forward
static void motor_spin_left(uint32_t speed = TURN_SPEED) {
    uint32_t lr = (speed > LEFT_TURN_BIAS) ? speed - LEFT_TURN_BIAS : 0;
    uint32_t rf = speed + LEFT_TURN_BIAS;
    if (rf > PWM_WRAP) rf = PWM_WRAP;
    _set_left(0, lr);
    _set_right(rf, 0);
    enc.set_left_dir(-1);
    enc.set_right_dir(1);
}

// Spin right: left wheel forward, right wheel reverse
static void motor_spin_right(uint32_t speed = TURN_SPEED) {
    uint32_t lf = (speed > RIGHT_TURN_BIAS) ? speed - RIGHT_TURN_BIAS : 0;
    uint32_t rr = speed + RIGHT_TURN_BIAS;
    if (rr > PWM_WRAP) rr = PWM_WRAP;
    _set_left(lf, 0);
    _set_right(0, rr);
    enc.set_left_dir(1);
    enc.set_right_dir(-1);
}

// =============================================================================
// Status print
// =============================================================================
static void print_status(const char* label) {
    int32_t L    = enc.left_ticks();
    int32_t R    = enc.right_ticks();
    int32_t diff = L - R;
    printf("[%-18s] L=%6ld R=%6ld diff=%5ld | "
           "x=%7.1fmm y=%7.1fmm th=%7.1fdeg dist=%7.1fmm\n",
           label, (long)L, (long)R, (long)diff,
           enc.x(), enc.y(),
           enc.theta() * 180.0f / (float)M_PI,
           enc.distance_to_origin());
    stdio_flush();
}

// =============================================================================
// drive_ticks — proportional trim, mirrors Finn's drive_ticks()
// =============================================================================
static void drive_ticks(int32_t ticks,
                        uint32_t speed = DRIVE_SPEED,
                        const char* label = "drive") {
    int32_t start_left  = enc.left_ticks();
    int32_t start_right = enc.right_ticks();
    motor_forward(speed, speed);

    while (true) {
        enc.update_odometry();
        int32_t delta_left  = enc.left_ticks()  - start_left;
        int32_t delta_right = enc.right_ticks() - start_right;
        int32_t traveled    = abs((delta_left + delta_right) / 2);
        if (traveled >= ticks) break;

        int32_t tick_error = delta_right - delta_left;
        int32_t correction = tick_error * TRIM_KP;

        int32_t l_cmd = (int32_t)speed + correction;
        int32_t r_cmd = (int32_t)speed - correction;
        if (l_cmd < 0) l_cmd = 0;
        if (r_cmd < 0) r_cmd = 0;
        if (l_cmd > (int32_t)PWM_WRAP) l_cmd = (int32_t)PWM_WRAP;
        if (r_cmd > (int32_t)PWM_WRAP) r_cmd = (int32_t)PWM_WRAP;

        motor_forward((uint32_t)l_cmd, (uint32_t)r_cmd);
        print_status(label);
        sleep_ms(LOOP_MS);
    }

    motor_stop();
    sleep_ms(300);
}

// =============================================================================
// rotate_to — mirrors Finn's rotate_to()
// =============================================================================
static void rotate_to(float target_theta,
                      uint32_t speed = TURN_SPEED,
                      const char* label = "rotate") {
    while (true) {
        enc.update_odometry();
        float err = WheelEncoder::normalize_angle(target_theta - enc.theta());
        if (fabsf(err) < ANGLE_TOL) break;

        if (err > 0.0f)
            motor_spin_left(speed);
        else
            motor_spin_right(speed);

        print_status(label);
        sleep_ms(LOOP_MS);
    }
    motor_stop();
    sleep_ms(300);
}

static void rotate_by(float degrees, uint32_t speed = TURN_SPEED) {
    char label[24];
    snprintf(label, sizeof(label), "%c %.0f deg",
             degrees > 0.0f ? 'L' : 'R', fabsf(degrees));
    float target = WheelEncoder::normalize_angle(
        enc.theta() + degrees * (float)M_PI / 180.0f);
    rotate_to(target, speed, label);
}

// =============================================================================
// drive_to_origin — mirrors Finn's rev 2 drive_to_origin()
// =============================================================================
static void drive_to_origin(uint32_t speed = DRIVE_SPEED) {
    printf("\n-- Returning to origin ------------------------------------------\n");
    stdio_flush();
    rotate_to(enc.angle_to_origin(), TURN_SPEED, "face origin");

    int32_t start_left    = enc.left_ticks();
    int32_t start_right   = enc.right_ticks();
    float   last_progress = enc.distance_to_origin();
    int     stuck_count   = 0;

    motor_forward(speed, speed);

    while (enc.distance_to_origin() > DIST_TOL) {
        enc.update_odometry();

        int32_t delta_left  = enc.left_ticks()  - start_left;
        int32_t delta_right = enc.right_ticks() - start_right;
        int32_t tick_error  = delta_right - delta_left;
        int32_t correction  = tick_error * TRIM_KP;

        float heading_err  = WheelEncoder::normalize_angle(
                                 enc.angle_to_origin() - enc.theta());
        float current_dist = enc.distance_to_origin();

        uint32_t approach_speed = (current_dist < 150.0f) ? speed / 2 : speed;

        if (fabsf(last_progress - current_dist) < 2.0f) {
            stuck_count++;
        } else {
            stuck_count   = 0;
            last_progress = current_dist;
        }
        if (stuck_count > 50) {
            printf("  >>> stuck near origin, stopping\n");
            stdio_flush();
            break;
        }

        if (fabsf(heading_err) > 0.12f) {
            motor_stop();
            rotate_to(enc.angle_to_origin(), TURN_SPEED, "correct heading");
            start_left  = enc.left_ticks();
            start_right = enc.right_ticks();
            motor_forward(approach_speed, approach_speed);
        } else {
            int32_t l_cmd = (int32_t)approach_speed + correction;
            int32_t r_cmd = (int32_t)approach_speed - correction;
            if (l_cmd < 0) l_cmd = 0;
            if (r_cmd < 0) r_cmd = 0;
            if (l_cmd > (int32_t)PWM_WRAP) l_cmd = (int32_t)PWM_WRAP;
            if (r_cmd > (int32_t)PWM_WRAP) r_cmd = (int32_t)PWM_WRAP;
            motor_forward((uint32_t)l_cmd, (uint32_t)r_cmd);
        }

        print_status("return");
        sleep_ms(LOOP_MS);
    }

    motor_stop();
    sleep_ms(300);
    rotate_to(0.0f, TURN_SPEED, "face start hdg");
}

// =============================================================================
// run — mirrors Finn's run()
// =============================================================================
static void run() {
    enc.reset_ticks();
    enc.reset_pose();

    const int32_t ticks_500mm = (int32_t)(500.0f / MM_PER_TICK);
    const int32_t ticks_300mm = (int32_t)(300.0f / MM_PER_TICK);
    const int32_t ticks_200mm = (int32_t)(200.0f / MM_PER_TICK);

    printf("======================================================================\n");
    printf("NAV TEST — origin set at current position\n");
    printf("Motor: L_FWD=GP%d L_REV=GP%d R_FWD=GP%d R_REV=GP%d\n",
           MOTOR_L_PIN_IN3, MOTOR_L_PIN_IN4, MOTOR_R_PIN_IN1, MOTOR_R_PIN_IN2);
    printf("Enc:   LEFT=GP%d RIGHT=GP%d\n", ENC_PIN_LEFT, ENC_PIN_RIGHT);
    printf("WHEEL_DIAM=%.1fmm  WHEELBASE=%.1fmm  MM_PER_TICK=%.4f\n",
           WHEEL_DIAM_MM, WHEELBASE_MM, MM_PER_TICK);
    printf("STRAIGHT_BIAS=%u  L_TURN=%u  R_TURN=%u  TRIM_KP=%d  LOOP=%dms\n",
           STRAIGHT_BIAS, LEFT_TURN_BIAS, RIGHT_TURN_BIAS, TRIM_KP, LOOP_MS);
    printf("ticks_500=%ld  ticks_300=%ld  ticks_200=%ld\n",
           (long)ticks_500mm, (long)ticks_300mm, (long)ticks_200mm);
    printf("======================================================================\n\n");
    stdio_flush();

    printf("-- [1] Forward 500mm --------------------------------------------\n");
    drive_ticks(ticks_500mm, DRIVE_SPEED, "fwd 500mm");
    print_status("after fwd 500");

    printf("\n-- [2] Right 90 deg ---------------------------------------------\n");
    rotate_by(-90.0f);
    print_status("after R 90");

    printf("\n-- [3] Forward 300mm --------------------------------------------\n");
    drive_ticks(ticks_300mm, DRIVE_SPEED, "fwd 300mm");
    print_status("after fwd 300");

    printf("\n-- [4] Left 135 deg ---------------------------------------------\n");
    rotate_by(135.0f);
    print_status("after L 135");

    printf("\n-- [5] Forward 200mm --------------------------------------------\n");
    drive_ticks(ticks_200mm, DRIVE_SPEED, "fwd 200mm");
    print_status("after fwd 200");

    printf("\n-- [6] Right 45 deg ---------------------------------------------\n");
    rotate_by(-45.0f);
    print_status("after R 45");

    drive_to_origin();
    print_status("FINAL");

    printf("\n======================================================================\n");
    printf("Final distance from origin: %.1f mm\n", enc.distance_to_origin());
    printf("Final x=%.1fmm  y=%.1fmm  theta=%.1fdeg\n",
           enc.x(), enc.y(), enc.theta() * 180.0f / (float)M_PI);
    printf("======================================================================\n");
    stdio_flush();
}

// =============================================================================
// main
// =============================================================================
int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Pull motor pins LOW immediately — prevents float-driven spin at boot
    gpio_init(MOTOR_L_PIN_IN3); gpio_set_dir(MOTOR_L_PIN_IN3, GPIO_OUT); gpio_put(MOTOR_L_PIN_IN3, 0);
    gpio_init(MOTOR_L_PIN_IN4); gpio_set_dir(MOTOR_L_PIN_IN4, GPIO_OUT); gpio_put(MOTOR_L_PIN_IN4, 0);
    gpio_init(MOTOR_R_PIN_IN1); gpio_set_dir(MOTOR_R_PIN_IN1, GPIO_OUT); gpio_put(MOTOR_R_PIN_IN1, 0);
    gpio_init(MOTOR_R_PIN_IN2); gpio_set_dir(MOTOR_R_PIN_IN2, GPIO_OUT); gpio_put(MOTOR_R_PIN_IN2, 0);

    // Wait up to 3s for USB CDC — battery-safe, won't block forever
    uint32_t usb_start = time_us_32();
    while (!stdio_usb_connected()) {
        if ((time_us_32() - usb_start) > 3000000u) break;
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    sleep_ms(200);

    printf("\n========================================\n");
    printf("  Encoder Nav Test\n");
    printf("  L_FWD=GP%d  L_REV=GP%d\n", MOTOR_L_PIN_IN3, MOTOR_L_PIN_IN4);
    printf("  R_FWD=GP%d  R_REV=GP%d\n", MOTOR_R_PIN_IN1, MOTOR_R_PIN_IN2);
    printf("  ENC_L=GP%d  ENC_R=GP%d\n", ENC_PIN_LEFT, ENC_PIN_RIGHT);
    printf("========================================\n\n");
    stdio_flush();

    // ── Motor PWM init ────────────────────────────────────────────────────────
    pwm_pin_init(MOTOR_L_PIN_IN3);
    pwm_pin_init(MOTOR_L_PIN_IN4);
    pwm_pin_init(MOTOR_R_PIN_IN1);
    pwm_pin_init(MOTOR_R_PIN_IN2);
    motor_stop();

    // ── Encoder init ──────────────────────────────────────────────────────────
    enc.init();

    printf("System ready — starting nav route in 2 seconds...\n");
    printf("MM_PER_TICK = %.4f mm\n", MM_PER_TICK);
    stdio_flush();
    gpio_put(LED_PIN, 1);
    sleep_ms(2000);
    gpio_put(LED_PIN, 0);

    run();

    motor_stop();
    printf("\nMotors stopped. Test complete.\n");
    stdio_flush();
    gpio_put(LED_PIN, 1);

    while (true) tight_loop_contents();
}