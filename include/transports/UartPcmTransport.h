#pragma once

#include <Arduino.h>

#include "transports/IAudioTransport.h"

class UartPcmTransport : public IAudioTransport {
public:
  struct Config {
    Stream* stream = &Serial;
    HardwareSerial* hardwareSerial = nullptr;
    uint32_t baudRate = 921600;
    int rxPin = -1;
    int txPin = -1;
  };

  UartPcmTransport();
  explicit UartPcmTransport(const Config& config);
  ~UartPcmTransport() override;

  bool begin() override;
  bool sendPcm16(const int16_t* samples,
                 size_t sampleCount,
                 uint32_t sampleRateHz,
                 uint64_t timestampUs) override;
  bool sendTaggedPcm16(uint8_t sourceId,
                       const int16_t* samples,
                       size_t sampleCount,
                       uint32_t sampleRateHz,
                       uint64_t timestampUs) override;
  bool sendSyncedTaggedPcm16(uint8_t sourceId,
                             uint32_t frameId,
                             const int16_t* samples,
                             size_t sampleCount,
                             uint32_t sampleRateHz,
                             uint64_t timestampUs) override;
  void end() override;

private:
  Config _config;
  bool _started = false;
  uint32_t _sequence = 0;
};
