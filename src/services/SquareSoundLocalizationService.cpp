#include "services/SquareSoundLocalizationService.h"

#include <math.h>
#include <stdlib.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;

// Układ współrzędnych macierzy: +X w prawo, +Y w stronę górnej pary.
// Kolejność kanałów: top-left, top-right, bottom-left, bottom-right.
constexpr float kNormalizedPositions[4][2] = {
    {-0.5f, 0.5f},
    {0.5f, 0.5f},
    {-0.5f, -0.5f},
    {0.5f, -0.5f},
};

}  // namespace

SquareSoundLocalizationService::SquareSoundLocalizationService(
    Sph0645DualMicDriver& diagonalAMics,
    Sph0645DualMicDriver& diagonalBMics,
    const Config& config)
    : FreeRtosService("SquareSoundDoA", 12288, 2, 1),
      _diagonalAMics(diagonalAMics),
      _diagonalBMics(diagonalBMics),
      _config(config),
      _motionScoutDetector(config.motionScoutConfig) {}

bool SquareSoundLocalizationService::takeLatestAngle(float& angleDegrees) {
  if (!_angleAvailable) {
    return false;
  }

  angleDegrees = _latestAngleDegrees;
  _angleAvailable = false;
  return true;
}

bool SquareSoundLocalizationService::onStart() {
  if (_config.chunkSamples == 0 ||
      _config.sampleRateHz == 0 ||
      _config.arrayWidthMeters <= 0.0f ||
      _config.arrayHeightMeters <= 0.0f ||
      _config.speedOfSoundMetersPerSecond <= 0.0f ||
      _config.maximumResidualMeters <= 0.0f ||
      _config.smoothingWindow == 0 ||
      _config.smoothingWindow > kMaximumSmoothingWindow) {
    return false;
  }

  _fftSize = nextPowerOfTwo(_config.chunkSamples * 2);
  if (_fftSize > kMaximumFftSize || !allocateBuffers()) {
    onStop();
    return false;
  }

  // Każdy kontroler mierzy jedną przekątną własnym zegarem. Algorytm nie
  // koreluje kanałów należących do różnych kontrolerów I2S.
  if (!_diagonalAMics.begin() || !_diagonalBMics.begin()) {
    onStop();
    return false;
  }

  _angleAvailable = false;
  _lastAnalysisAtMs = 0;
  _smoothingIndex = 0;
  _smoothingCount = 0;
  _motionScoutDetector.reset();
  for (size_t i = 0; i < kMaximumSmoothingWindow; ++i) {
    _angleCosines[i] = 0.0f;
    _angleSines[i] = 0.0f;
    _angleWeights[i] = 0.0f;
  }
  return true;
}

void SquareSoundLocalizationService::onLoop() {
  size_t diagonalASamples = 0;
  size_t diagonalBSamples = 0;
  // I2S1: LEFT=top-left, RIGHT=bottom-right.
  // I2S0: LEFT=top-right, RIGHT=bottom-left.
  const bool diagonalAOk = _diagonalAMics.read(
      _audio[0], _audio[3], _config.chunkSamples, diagonalASamples);
  const bool diagonalBOk = _diagonalBMics.read(
      _audio[1], _audio[2], _config.chunkSamples, diagonalBSamples);

  if (!diagonalAOk || !diagonalBOk ||
      diagonalASamples == 0 || diagonalBSamples == 0) {
    if (_config.motionScoutDetectionEnabled) {
      _motionScoutDetector.reset();
    }
    sleepMs(2);
    return;
  }

  const size_t sampleCount = (diagonalASamples < diagonalBSamples)
                                 ? diagonalASamples
                                 : diagonalBSamples;

  float maximumRms = 0.0f;
  for (size_t microphone = 0; microphone < kMicrophoneCount; ++microphone) {
    const float rms = calculateRms(_audio[microphone], sampleCount);
    if (rms > maximumRms) {
      maximumRms = rms;
    }
  }

  if (maximumRms < _config.minimumRms) {
    if (_config.motionScoutDetectionEnabled) {
      _motionScoutDetector.reset();
    }
    return;
  }

  const uint32_t nowMs = millis();
  if (_lastAnalysisAtMs != 0 &&
      nowMs - _lastAnalysisAtMs < _config.reportIntervalMs) {
    return;
  }
  _lastAnalysisAtMs = nowMs;

  prepareSpectra(sampleCount);

  if (_config.motionScoutDetectionEnabled) {
    float microphoneSnrDb[kMicrophoneCount] = {};
    for (size_t microphone = 0; microphone < kMicrophoneCount; ++microphone) {
      microphoneSnrDb[microphone] = _motionScoutDetector.calculateSnrDb(
          _spectrumReal[microphone],
          _spectrumImaginary[microphone],
          _fftSize,
          _config.sampleRateHz);
    }

    const MotionScoutDetector::Result detection =
        _motionScoutDetector.update(microphoneSnrDb, kMicrophoneCount);
    Serial.printf("SCOUT snr=%.2f matching=%u detected=%u\n",
                  detection.averageSnrDb,
                  static_cast<unsigned int>(detection.matchingMicrophones),
                  detection.detected ? 1U : 0U);
    if (!detection.detected) {
      return;
    }
  }

  float angleDegrees = 0.0f;
  float quality = 0.0f;
  if (estimateAngle(sampleCount, angleDegrees, quality)) {
    publishAngle(angleDegrees, quality);
  }
}

void SquareSoundLocalizationService::onStop() {
  _diagonalBMics.end();
  _diagonalAMics.end();
  releaseBuffers();
  _angleAvailable = false;
}

size_t SquareSoundLocalizationService::nextPowerOfTwo(size_t value) {
  size_t result = 1;
  while (result < value) {
    result <<= 1U;
  }
  return result;
}

void SquareSoundLocalizationService::fft(float* real,
                                          float* imaginary,
                                          size_t size,
                                          bool inverse) {
  for (size_t i = 1, j = 0; i < size; ++i) {
    size_t bit = size >> 1U;
    for (; j & bit; bit >>= 1U) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      const float realTemporary = real[i];
      const float imaginaryTemporary = imaginary[i];
      real[i] = real[j];
      imaginary[i] = imaginary[j];
      real[j] = realTemporary;
      imaginary[j] = imaginaryTemporary;
    }
  }

  for (size_t length = 2; length <= size; length <<= 1U) {
    const float angle = (inverse ? 2.0f : -2.0f) * kPi / static_cast<float>(length);
    const float stepReal = cosf(angle);
    const float stepImaginary = sinf(angle);

    for (size_t offset = 0; offset < size; offset += length) {
      float twiddleReal = 1.0f;
      float twiddleImaginary = 0.0f;
      const size_t halfLength = length >> 1U;

      for (size_t index = 0; index < halfLength; ++index) {
        const size_t evenIndex = offset + index;
        const size_t oddIndex = evenIndex + halfLength;
        const float oddReal = real[oddIndex] * twiddleReal -
                              imaginary[oddIndex] * twiddleImaginary;
        const float oddImaginary = real[oddIndex] * twiddleImaginary +
                                   imaginary[oddIndex] * twiddleReal;

        real[oddIndex] = real[evenIndex] - oddReal;
        imaginary[oddIndex] = imaginary[evenIndex] - oddImaginary;
        real[evenIndex] += oddReal;
        imaginary[evenIndex] += oddImaginary;

        const float nextTwiddleReal = twiddleReal * stepReal -
                                      twiddleImaginary * stepImaginary;
        twiddleImaginary = twiddleReal * stepImaginary +
                           twiddleImaginary * stepReal;
        twiddleReal = nextTwiddleReal;
      }
    }
  }

  if (inverse) {
    const float scale = 1.0f / static_cast<float>(size);
    for (size_t i = 0; i < size; ++i) {
      real[i] *= scale;
      imaginary[i] *= scale;
    }
  }
}

float SquareSoundLocalizationService::calculateRms(const int16_t* samples,
                                                    size_t count) {
  double energy = 0.0;
  for (size_t i = 0; i < count; ++i) {
    const double sample = static_cast<double>(samples[i]);
    energy += sample * sample;
  }
  return static_cast<float>(sqrt(energy / static_cast<double>(count)));
}

bool SquareSoundLocalizationService::allocateBuffers() {
  for (size_t microphone = 0; microphone < kMicrophoneCount; ++microphone) {
    _audio[microphone] =
        static_cast<int16_t*>(malloc(_config.chunkSamples * sizeof(int16_t)));
    _spectrumReal[microphone] =
        static_cast<float*>(malloc(_fftSize * sizeof(float)));
    _spectrumImaginary[microphone] =
        static_cast<float*>(malloc(_fftSize * sizeof(float)));
  }
  _workReal = static_cast<float*>(malloc(_fftSize * sizeof(float)));
  _workImaginary = static_cast<float*>(malloc(_fftSize * sizeof(float)));

  if (_workReal == nullptr || _workImaginary == nullptr) {
    return false;
  }
  for (size_t microphone = 0; microphone < kMicrophoneCount; ++microphone) {
    if (_audio[microphone] == nullptr ||
        _spectrumReal[microphone] == nullptr ||
        _spectrumImaginary[microphone] == nullptr) {
      return false;
    }
  }
  return true;
}

void SquareSoundLocalizationService::releaseBuffers() {
  for (size_t microphone = 0; microphone < kMicrophoneCount; ++microphone) {
    free(_audio[microphone]);
    free(_spectrumReal[microphone]);
    free(_spectrumImaginary[microphone]);
    _audio[microphone] = nullptr;
    _spectrumReal[microphone] = nullptr;
    _spectrumImaginary[microphone] = nullptr;
  }
  free(_workReal);
  free(_workImaginary);
  _workReal = nullptr;
  _workImaginary = nullptr;
  _fftSize = 0;
}

void SquareSoundLocalizationService::prepareSpectra(size_t sampleCount) {
  for (size_t microphone = 0; microphone < kMicrophoneCount; ++microphone) {
    for (size_t i = 0; i < _fftSize; ++i) {
      if (i < sampleCount) {
        const float window = 0.5f -
            0.5f * cosf(2.0f * kPi * static_cast<float>(i) /
                        static_cast<float>(sampleCount - 1));
        _spectrumReal[microphone][i] =
            static_cast<float>(_audio[microphone][i]) * window;
      } else {
        _spectrumReal[microphone][i] = 0.0f;
      }
      _spectrumImaginary[microphone][i] = 0.0f;
    }
    fft(_spectrumReal[microphone], _spectrumImaginary[microphone], _fftSize, false);
  }
}

SquareSoundLocalizationService::PairDelay
SquareSoundLocalizationService::estimatePairDelay(uint8_t first,
                                                   uint8_t second,
                                                   size_t sampleCount) {
  PairDelay result{first, second, 0.0f, 0.0f, false};

  for (size_t bin = 0; bin < _fftSize; ++bin) {
    const float firstReal = _spectrumReal[first][bin];
    const float firstImaginary = _spectrumImaginary[first][bin];
    const float secondReal = _spectrumReal[second][bin];
    const float secondImaginary = _spectrumImaginary[second][bin];
    float crossReal = firstReal * secondReal + firstImaginary * secondImaginary;
    float crossImaginary = firstImaginary * secondReal - firstReal * secondImaginary;
    const float magnitude = sqrtf(crossReal * crossReal + crossImaginary * crossImaginary);
    if (magnitude > 1.0e-12f) {
      crossReal /= magnitude;
      crossImaginary /= magnitude;
    } else {
      crossReal = 0.0f;
      crossImaginary = 0.0f;
    }
    _workReal[bin] = crossReal;
    _workImaginary[bin] = crossImaginary;
  }

  fft(_workReal, _workImaginary, _fftSize, true);

  const float deltaX = (kNormalizedPositions[first][0] -
                        kNormalizedPositions[second][0]) * _config.arrayWidthMeters;
  const float deltaY = (kNormalizedPositions[first][1] -
                        kNormalizedPositions[second][1]) * _config.arrayHeightMeters;
  const float distance = sqrtf(deltaX * deltaX + deltaY * deltaY);
  int maximumLag = static_cast<int>(ceilf(
      distance * static_cast<float>(_config.sampleRateHz) /
      _config.speedOfSoundMetersPerSecond));
  if (maximumLag < 1) {
    maximumLag = 1;
  }
  if (maximumLag >= static_cast<int>(sampleCount)) {
    maximumLag = static_cast<int>(sampleCount) - 1;
  }

  int bestLag = 0;
  float bestValue = -1.0e30f;
  float meanAbsolute = 0.0f;
  for (int lag = -maximumLag; lag <= maximumLag; ++lag) {
    const size_t index = (lag >= 0) ? static_cast<size_t>(lag)
                                    : _fftSize - static_cast<size_t>(-lag);
    const float value = _workReal[index];
    meanAbsolute += fabsf(value);
    if (value > bestValue) {
      bestValue = value;
      bestLag = lag;
    }
  }
  meanAbsolute /= static_cast<float>(2 * maximumLag + 1);

  const auto correlationAtLag = [this](int lag) {
    const size_t index = (lag >= 0) ? static_cast<size_t>(lag)
                                    : _fftSize - static_cast<size_t>(-lag);
    return _workReal[index];
  };

  float fractionalLag = static_cast<float>(bestLag);
  if (bestLag > -maximumLag && bestLag < maximumLag) {
    const float previous = correlationAtLag(bestLag - 1);
    const float center = correlationAtLag(bestLag);
    const float next = correlationAtLag(bestLag + 1);
    const float denominator = previous - 2.0f * center + next;
    if (fabsf(denominator) > 1.0e-9f) {
      float correction = 0.5f * (previous - next) / denominator;
      if (correction > 0.5f) correction = 0.5f;
      if (correction < -0.5f) correction = -0.5f;
      fractionalLag += correction;
    }
  }

  result.delaySeconds = fractionalLag / static_cast<float>(_config.sampleRateHz);
  result.peakRatio = bestValue / (meanAbsolute + 1.0e-9f);
  result.valid = result.peakRatio >= _config.minimumPairPeakRatio;
  return result;
}

bool SquareSoundLocalizationService::estimateAngle(size_t sampleCount,
                                                   float& angleDegrees,
                                                   float& quality) {

  // Obie pary należą w całości do jednego kontrolera I2S:
  // A = top-left - bottom-right, B = top-right - bottom-left.
  const PairDelay diagonalA = estimatePairDelay(0, 3, sampleCount);
  const PairDelay diagonalB = estimatePairDelay(1, 2, sampleCount);
  if (!diagonalA.valid || !diagonalB.valid) {
    return false;
  }

  const float measuredPathA =
      _config.speedOfSoundMetersPerSecond * diagonalA.delaySeconds;
  const float measuredPathB =
      _config.speedOfSoundMetersPerSecond * diagonalB.delaySeconds;
  const float weightA = (diagonalA.peakRatio < 20.0f)
                            ? diagonalA.peakRatio
                            : 20.0f;
  const float weightB = (diagonalB.peakRatio < 20.0f)
                            ? diagonalB.peakRatio
                            : 20.0f;
  const float weightSum = weightA + weightB;

  const auto costAtDegrees = [this,
                              measuredPathA,
                              measuredPathB,
                              weightA,
                              weightB,
                              weightSum](float degrees) {
    const float radians = degrees * kPi / 180.0f;
    const float sourceX = cosf(radians);
    const float sourceY = sinf(radians);

    // Wektor propagacji jest przeciwny do wektora wskazującego źródło.
    const float predictedPathA =
        _config.arrayWidthMeters * sourceX -
        _config.arrayHeightMeters * sourceY;
    const float predictedPathB =
        -_config.arrayWidthMeters * sourceX -
        _config.arrayHeightMeters * sourceY;
    const float residualA = measuredPathA - predictedPathA;
    const float residualB = measuredPathB - predictedPathB;

    return (weightA * residualA * residualA +
            weightB * residualB * residualB) /
           weightSum;
  };

  float bestAngleDegrees = 0.0f;
  float bestCost = costAtDegrees(0.0f);
  for (int degrees = 1; degrees < 360; ++degrees) {
    const float cost = costAtDegrees(static_cast<float>(degrees));
    if (cost < bestCost) {
      bestCost = cost;
      bestAngleDegrees = static_cast<float>(degrees);
    }
  }

  // Doprecyzowanie wyniku do 0,05 stopnia wokół najlepszego pełnego stopnia.
  angleDegrees = bestAngleDegrees;
  for (int step = -20; step <= 20; ++step) {
    const float candidate = bestAngleDegrees + static_cast<float>(step) * 0.05f;
    const float cost = costAtDegrees(candidate);
    if (cost < bestCost) {
      bestCost = cost;
      angleDegrees = candidate;
    }
  }

  while (angleDegrees < 0.0f) angleDegrees += 360.0f;
  while (angleDegrees >= 360.0f) angleDegrees -= 360.0f;

  const float residualMeters = sqrtf(bestCost);
  if (residualMeters > _config.maximumResidualMeters) {
    return false;
  }

  const float weakerPeak = (diagonalA.peakRatio < diagonalB.peakRatio)
                               ? diagonalA.peakRatio
                               : diagonalB.peakRatio;
  quality = weakerPeak /
            (1.0f + residualMeters / _config.maximumResidualMeters);
  return true;
}

void SquareSoundLocalizationService::publishAngle(float angleDegrees,
                                                  float quality) {
  const float radians = angleDegrees * kPi / 180.0f;
  const float weight = (quality > 0.0f) ? quality : 1.0f;
  _angleCosines[_smoothingIndex] = cosf(radians);
  _angleSines[_smoothingIndex] = sinf(radians);
  _angleWeights[_smoothingIndex] = weight;
  _smoothingIndex = (_smoothingIndex + 1) % _config.smoothingWindow;
  if (_smoothingCount < _config.smoothingWindow) {
    ++_smoothingCount;
  }

  float directionX = 0.0f;
  float directionY = 0.0f;
  for (size_t i = 0; i < _smoothingCount; ++i) {
    directionX += _angleWeights[i] * _angleCosines[i];
    directionY += _angleWeights[i] * _angleSines[i];
  }

  float smoothedAngle = atan2f(directionY, directionX) * 180.0f / kPi;
  if (smoothedAngle < 0.0f) {
    smoothedAngle += 360.0f;
  }
  _latestAngleDegrees = smoothedAngle;
  _angleAvailable = true;
}
