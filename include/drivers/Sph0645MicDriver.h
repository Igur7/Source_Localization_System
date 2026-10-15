#pragma once

#include <Arduino.h>
#include "driver/i2s.h"
#include "drivers/IMicDriver.h"

class Sph0645MicDriver : public IMicDriver {
public:
  enum class Channel {
    Left,
    Right
  };

  struct Config {
    i2s_port_t port = I2S_NUM_0;
    int bclkPin = 3;    // XIAO ESP32-C3 D1
    int wsPin = 2;      // XIAO ESP32-C3 D0
    int dataInPin = 4;  // XIAO ESP32-C3 D2
    Channel channel = Channel::Right;
    uint32_t sampleRateHz = 32000;
    size_t dmaBufferCount = 8;
    size_t dmaBufferLength = 128;
    size_t readFrameCount = 256;
    bool removeDcOffset = true;
    bool masterClock = true;
  };

  Sph0645MicDriver();
  explicit Sph0645MicDriver(const Config& config);
  ~Sph0645MicDriver() override;

  bool begin() override;
  bool read(int16_t* buffer, size_t capacity, size_t& outSamples) override;
  void end() override;

  int32_t lastDcOffset() const;
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
  int32_t _lastDcOffset = 0;
};
