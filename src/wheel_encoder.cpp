// =============================================================================
// wheel_encoder.cpp
// IRQ shim and init. All inline logic lives in wheel_encoder.h.
// =============================================================================

#include "wheel_encoder.h"
#include "hardware/gpio.h"

WheelEncoder* g_encoder_instance = nullptr;

static void encoder_irq_handler(uint gpio, uint32_t events) {
    if (!g_encoder_instance) return;
    if      (gpio == ENC_LEFT_PIN)  g_encoder_instance->_isr_left (gpio, events);
    else if (gpio == ENC_RIGHT_PIN) g_encoder_instance->_isr_right(gpio, events);
}

void WheelEncoder::init() {
    g_encoder_instance = this;

    gpio_init(ENC_LEFT_PIN);
    gpio_set_dir(ENC_LEFT_PIN, GPIO_IN);
    gpio_pull_up(ENC_LEFT_PIN);

    gpio_init(ENC_RIGHT_PIN);
    gpio_set_dir(ENC_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(ENC_RIGHT_PIN);

    // One shared callback dispatched by GPIO number — same pattern as MicroPython
    // enc_left_pin.irq(trigger=Pin.IRQ_FALLING, handler=_on_left_tick)
    gpio_set_irq_enabled_with_callback(ENC_LEFT_PIN,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &encoder_irq_handler);
    gpio_set_irq_enabled_with_callback(ENC_RIGHT_PIN,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &encoder_irq_handler);
}