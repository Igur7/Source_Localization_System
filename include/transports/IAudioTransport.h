#pragma once

#include <Arduino.h>

class IAudioTransport {
public:
  virtual ~IAudioTransport() = default;

  virtual bool begin() = 0;
  virtual bool sendPcm16(const int16_t* samples,
                         size_t sampleCount,
                         uint32_t sampleRateHz,
                         uint64_t timestampUs) = 0;
  virtual bool sendTaggedPcm16(uint8_t sourceId,
                               const int16_t* samples,
                               size_t sampleCount,
                               uint32_t sampleRateHz,
                               uint64_t timestampUs);
  virtual bool sendSyncedTaggedPcm16(uint8_t sourceId,
                                     uint32_t frameId,
                                     const int16_t* samples,
                                     size_t sampleCount,
                                     uint32_t sampleRateHz,
                                     uint64_t timestampUs);
  virtual void end() = 0;
};
