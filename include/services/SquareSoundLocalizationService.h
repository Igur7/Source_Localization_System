#pragma once

#include <Arduino.h>

#include "core/FreeRtosService.h"
#include "drivers/Sph0645DualMicDriver.h"
#include "dsp/MotionScoutDetector.h"


class SquareSoundLocalizationService : public FreeRtosService {
public:
  struct Config {
    uint32_t sampleRateHz = 48000;
    size_t chunkSamples = 512;
    float arrayWidthMeters = 0.134f;
    float arrayHeightMeters = 0.140f;
    float speedOfSoundMetersPerSecond = 343.0f;
    float minimumRms = 80.0f;
    float minimumPairPeakRatio = 1.5f;
    float maximumResidualMeters = 0.03f;
    size_t smoothingWindow = 5;
    uint32_t reportIntervalMs = 200;
    bool motionScoutDetectionEnabled = false;
    MotionScoutDetector::Config motionScoutConfig;
  };

  SquareSoundLocalizationService(Sph0645DualMicDriver& diagonalAMics,
                                 Sph0645DualMicDriver& diagonalBMics,
                                 const Config& config);

  bool takeLatestAngle(float& angleDegrees);

protected:
  bool onStart() override;
  void onLoop() override;
  void onStop() override;

private:
  static constexpr size_t kMicrophoneCount = 4;
  static constexpr size_t kMaximumFftSize = 2048;
  static constexpr size_t kMaximumSmoothingWindow = 16;

  struct PairDelay {
    uint8_t first;
    uint8_t second;
    float delaySeconds;
    float peakRatio;
    bool valid;
  };

  static size_t nextPowerOfTwo(size_t value);
  static void fft(float* real, float* imaginary, size_t size, bool inverse);
  static float calculateRms(const int16_t* samples, size_t count);

  bool allocateBuffers();
  void releaseBuffers();
  void prepareSpectra(size_t sampleCount);
  PairDelay estimatePairDelay(uint8_t first, uint8_t second, size_t sampleCount);
  bool estimateAngle(size_t sampleCount, float& angleDegrees, float& quality);
  void publishAngle(float angleDegrees, float quality);

  Sph0645DualMicDriver& _diagonalAMics;
  Sph0645DualMicDriver& _diagonalBMics;
  Config _config;
  MotionScoutDetector _motionScoutDetector;
  size_t _fftSize = 0;
  int16_t* _audio[kMicrophoneCount] = {};
  float* _spectrumReal[kMicrophoneCount] = {};
  float* _spectrumImaginary[kMicrophoneCount] = {};
  float* _workReal = nullptr;
  float* _workImaginary = nullptr;
  uint32_t _lastAnalysisAtMs = 0;
  float _angleCosines[kMaximumSmoothingWindow] = {};
  float _angleSines[kMaximumSmoothingWindow] = {};
  float _angleWeights[kMaximumSmoothingWindow] = {};
  size_t _smoothingIndex = 0;
  size_t _smoothingCount = 0;
  volatile bool _angleAvailable = false;
  volatile float _latestAngleDegrees = 0.0f;
};
