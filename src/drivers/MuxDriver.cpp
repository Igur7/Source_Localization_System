#include "drivers/MuxDriver.h"
#include <cmath>

void MuxDriver::begin() {
    // Zablokuj multiplekser podczas konfiguracji.
    digitalWrite(kEnablePin, HIGH);
    pinMode(kEnablePin, OUTPUT);

    for (uint8_t pin : _selectPins) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }
}

void MuxDriver::showAngle(float angleDegrees) {
    if (!std::isfinite(angleDegrees)) {
        disable();
        return;
    }

    selectChannel(angleToChannel(angleDegrees));
}

uint8_t MuxDriver::angleToChannel(float angleDegrees) const {
    // Zawijanie kąta, np. -90° -> 270°, 370° -> 10°.
    float angle = std::fmod(angleDegrees, 360.0f);

    if (angle < 0.0f) {
        angle += 360.0f;
    }

    // Najbliższy sektor: 360° / 16 = 22,5°.
    const uint8_t sector =
        static_cast<uint8_t>((angle + 11.25f) / 22.5f) % kLedCount;

    return _channelForSector[sector];
}

void MuxDriver::selectChannel(uint8_t channel) {
    // Zgaś LED podczas zmiany adresu.
    disable();

    if (channel >= kLedCount) {
        return;
    }

    for (uint8_t bit = 0; bit < 4; ++bit) {
        const bool state = (channel >> bit) & 1U;
        digitalWrite(_selectPins[bit], state ? HIGH : LOW);
    }

    // Połącz wybrany kanał z masą — zapal LED.
    digitalWrite(kEnablePin, LOW);
}

void MuxDriver::disable() {
    digitalWrite(kEnablePin, HIGH);
}