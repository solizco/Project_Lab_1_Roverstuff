#pragma once
#include "hardware/i2c.h"
#include <cstdint>
#include <cstring>

class TwoWire {
public:
    explicit TwoWire(i2c_inst_t* i2c)
        : _i2c(i2c), _addr(0), _bufLen(0), _rxLen(0), _rxIdx(0) {}

    void begin(uint32_t freq = 100000) {
        i2c_init(_i2c, freq);
    }

    void beginTransmission(uint8_t addr) {
        _addr   = addr;
        _bufLen = 0;
    }

    size_t write(uint8_t byte) {
        if (_bufLen < sizeof(_buf)) { _buf[_bufLen++] = byte; return 1; }
        return 0;
    }

    size_t write(const uint8_t* data, size_t len) {
        size_t n = 0;
        for (size_t i = 0; i < len; i++) n += write(data[i]);
        return n;
    }

    uint8_t endTransmission(bool stop = true) {
        if (_bufLen == 0) return 0;
        int ret = i2c_write_timeout_us(_i2c, _addr, _buf, _bufLen, !stop, 50000);
        uint8_t status = (ret == (int)_bufLen) ? 0 : 1;
        _bufLen = 0;
        return status;
    }

    uint8_t requestFrom(uint8_t addr, uint8_t len) {
        _rxLen = 0;
        _rxIdx = 0;
        if (len == 0 || len > sizeof(_rxBuf)) return 0;
        int ret = i2c_read_timeout_us(_i2c, addr, _rxBuf, len, false, 50000);
        if (ret > 0) _rxLen = (uint8_t)ret;
        return _rxLen;
    }

    int read() {
        if (_rxIdx < _rxLen) return _rxBuf[_rxIdx++];
        return -1;
    }

    int available() { return (int)(_rxLen - _rxIdx); }

private:
    i2c_inst_t* _i2c;
    uint8_t     _addr;
    uint8_t     _buf[128];
    uint8_t     _bufLen;
    uint8_t     _rxBuf[128];
    uint8_t     _rxLen;
    uint8_t     _rxIdx;
};