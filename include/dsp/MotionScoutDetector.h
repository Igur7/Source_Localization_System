#pragma once

#include <stddef.h>
#include <stdint.h>

class MotionScoutDetector {
public:
  struct Config {
    uint32_t targetLowHz = 2600;
    uint32_t targetHighHz = 3000;
    uint32_t lowerNoiseLowHz = 1800;
    uint32_t lowerNoiseHighHz = 2400;
    uint32_t upperNoiseLowHz = 3200;
    uint32_t upperNoiseHighHz = 4000;
    float minimumSnrDb = 10.0f;  // Signal-to-noise ratio in decibels.
    uint8_t requiredMicrophones = 3;
    uint8_t requiredConsecutiveFrames = 3;  // Frames confirming the alarm.
  };

  struct Result {
    bool detected = false;
    uint8_t matchingMicrophones = 0;
    float averageSnrDb = 0.0f;
  };

  explicit MotionScoutDetector(const Config& config);

  float calculateSnrDb(const float* spectrumReal,
                       const float* spectrumImaginary,
                       size_t fftSize,
                       uint32_t sampleRateHz) const;

  Result update(const float* microphoneSnrDb, size_t microphoneCount);
  void reset();

private:
  float calculateBandPower(const float* spectrumReal,
                           const float* spectrumImaginary,
                           size_t fftSize,
                           uint32_t sampleRateHz,
                           uint32_t lowHz,
                           uint32_t highHz) const;

  Config _config;
  uint8_t _consecutiveFrames = 0;
};
