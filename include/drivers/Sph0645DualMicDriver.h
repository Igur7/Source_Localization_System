#pragma once

#include <Arduino.h>
#include "driver/i2s.h"

class Sph0645DualMicDriver {
public:
  struct Config {
    i2s_port_t port = I2S_NUM_0;
    int bclkPin = 4;
    int wsPin = 5;
    int dataInPin = 6;
    bool masterClock = true;
    uint32_t sampleRateHz = 32000;
    size_t dmaBufferCount = 8;
    size_t dmaBufferLength = 128;
    size_t readFrameCount = 256;
    bool removeDcOffset = true;
  };

  Sph0645DualMicDriver();
  explicit Sph0645DualMicDriver(const Config& config);
  ~Sph0645DualMicDriver();

  bool begin();
  bool read(int16_t* leftBuffer,
            int16_t* rightBuffer,
            size_t capacity,
            size_t& outSamples);
  void end();

  int32_t lastLeftDcOffset() const;
  int32_t lastRightDcOffset() const;
  uint32_t sampleRateHz() const;

private:
  static constexpr int kRawShift = 14;
  static constexpr int kPcmShift = 2;

  static int16_t rawWordToPcm16(int32_t rawWord);
  static int32_t computeMean(const int16_t* buffer, size_t count);
  static void removeDcOffsetFromBuffer(int16_t* buffer, size_t count, int32_t mean);

  Config _config;
  int32_t* _rawBuffer = nullptr;
  size_t _rawWordCapacity = 0;
  bool _started = false;
  int32_t _lastLeftDcOffset = 0;
  int32_t _lastRightDcOffset = 0;
};
