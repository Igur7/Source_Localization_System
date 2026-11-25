#pragma once
#include <Arduino.h>

class MuxDriver {
public:
    MuxDriver() = default;
    ~MuxDriver() = default;

    void begin();
    void showAngle(float angleDegrees);
    void disable();

private:
    uint8_t angleToChannel(float angleDegrees) const;
    void selectChannel(uint8_t channel);

    static constexpr uint8_t kLedCount = 16;
    static constexpr uint8_t kEnablePin = 8;

    const uint8_t _selectPins[4] = {4, 5, 6, 7};

    // Kanały dla sektorów 0°, 22,5°, 45°...;
    // korekta fizycznego położenia pierścienia o 180° (8 LED).
    const uint8_t _channelForSector[kLedCount] = {
        8, 9, 10, 11, 12, 13, 14, 15,
        0, 1, 2, 3, 4, 5, 6, 7
    };
};
