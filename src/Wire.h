#pragma once
// pico_arduino_wire_shim.h
// Implements just enough of the Arduino TwoWire API for the Pololu
// VL53L0X and VL53L1X libraries to compile and run on the Pico SDK.
//
// Only the methods the Pololu libraries actually call are implemented:
//   beginTransmission(), write(), endTransmission(),
//   requestFrom(), read(), begin()
//
// Internal tx/rx buffers are 32 bytes — enough for all sensor register
// reads/writes the Pololu drivers perform (largest is ~14 bytes).
 
#include <cstdint>
#include <cstring>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
 
#define TWOWIRE_BUFFER_SIZE 32
 
class TwoWire {
public:
    // ── Constructor ───────────────────────────────────────────────────────────
    // Pass the pico-sdk i2c instance this bus wraps (i2c0 or i2c1).
    explicit TwoWire(i2c_inst_t* i2c_instance)
        : _i2c(i2c_instance),
          _addr(0),
          _tx_len(0),
          _rx_len(0),
          _rx_idx(0)
    {}
 
    // ── begin() ───────────────────────────────────────────────────────────────
    // Initialises the pico-sdk I2C peripheral at the requested baud rate.
    // gpio_set_function / gpio_pull_up should be called BEFORE begin() so the
    // correct physical pins are already assigned when i2c_init runs.
    void begin(uint32_t freq_hz = 100000) {
        i2c_init(_i2c, freq_hz);
    }
 
    // ── beginTransmission(addr) ───────────────────────────────────────────────
    // Records the target address and resets the transmit buffer.
    // Nothing is sent to the bus yet.
    void beginTransmission(uint8_t addr) {
        _addr   = addr;
        _tx_len = 0;
    }
 
    // ── write(byte) ───────────────────────────────────────────────────────────
    // Appends one byte to the transmit buffer.
    // Returns 1 on success, 0 if the buffer is full.
    size_t write(uint8_t byte) {
        if (_tx_len >= TWOWIRE_BUFFER_SIZE) return 0;
        _tx_buf[_tx_len++] = byte;
        return 1;
    }
 
    // ── write(buf, len) ───────────────────────────────────────────────────────
    // Appends multiple bytes to the transmit buffer.
    size_t write(const uint8_t* buf, size_t len) {
        size_t written = 0;
        for (size_t i = 0; i < len; i++) {
            if (write(buf[i]) == 0) break;
            written++;
        }
        return written;
    }
 
    // ── endTransmission(stop) ─────────────────────────────────────────────────
    // Flushes the transmit buffer to the bus.
    // stop = true  → send STOP condition after data (default, releases bus)
    // stop = false → send REPEATED START (no STOP), used before a read
    //
    // Return codes mirror Arduino Wire:
    //   0 = success
    //   1 = data too long (shouldn't happen given our callers)
    //   2 = NACK on address
    //   3 = NACK on data
    //   4 = other error
    uint8_t endTransmission(bool stop = true) {
        if (_tx_len == 0) return 0;
 
        // nostop = !stop → true means keep bus active (repeated start)
        int ret = i2c_write_timeout_us(_i2c, _addr, _tx_buf, _tx_len, !stop, 10000);
        _tx_len = 0;
 
        if (ret == PICO_ERROR_GENERIC) return 2; // NACK
        if (ret < 0)                  return 4; // other
        return 0;
    }
 
    // ── requestFrom(addr, count) ──────────────────────────────────────────────
    // Issues an I2C read of `count` bytes from `addr` and stores them in the
    // receive buffer.  Returns the number of bytes actually read.
    uint8_t requestFrom(uint8_t addr, uint8_t count) {
        if (count > TWOWIRE_BUFFER_SIZE) count = TWOWIRE_BUFFER_SIZE;
        int ret = i2c_read_timeout_us(_i2c, addr, _rx_buf, count, false, 10000);
        if (ret < 0) {
            _rx_len = 0;
            _rx_idx = 0;
            return 0;
        }
        _rx_len = (uint8_t)ret;
        _rx_idx = 0;
        return _rx_len;
    }
 
    // ── read() ────────────────────────────────────────────────────────────────
    // Returns the next byte from the receive buffer, or -1 if empty.
    int read() {
        if (_rx_idx >= _rx_len) return -1;
        return _rx_buf[_rx_idx++];
    }
 
    // ── available() ──────────────────────────────────────────────────────────
    // Returns how many unread bytes remain in the receive buffer.
    int available() {
        return (int)(_rx_len - _rx_idx);
    }
 
private:
    i2c_inst_t* _i2c;
 
    uint8_t _addr;
 
    uint8_t _tx_buf[TWOWIRE_BUFFER_SIZE];
    uint8_t _tx_len;
 
    uint8_t _rx_buf[TWOWIRE_BUFFER_SIZE];
    uint8_t _rx_len;
    uint8_t _rx_idx;
};


// Global Wire instance expected by Pololu libraries.
// Points to i2c0 by default — same bus used by the sensors.
// main.cpp calls Wire0.begin() to actually initialize the hardware;
// this global just satisfies the constructor reference in VL53L0X/VL53L1X.
#include "hardware/i2c.h"
extern "C" { }  // ensure C linkage compatibility

inline TwoWire& _get_wire_instance() {
    static TwoWire _Wire(i2c0);
    return _Wire;
}
#define Wire (_get_wire_instance())