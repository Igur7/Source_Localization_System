#include "dsp/MotionScoutDetector.h"
#include <math.h>

MotionScoutDetector::MotionScoutDetector(const Config& config)
    : _config(config) {}

void MotionScoutDetector::reset() {
  _consecutiveFrames = 0;
}

float MotionScoutDetector::calculateBandPower(const float* spectrumReal,
                                           const float* spectrumImaginary,
                                           size_t fftSize,
                                           uint32_t sampleRateHz,
                                           uint32_t lowHz,
                                           uint32_t highHz) const {
    if (spectrumReal == nullptr || spectrumImaginary == nullptr
        || fftSize < 2 || sampleRateHz == 0 || lowHz >= highHz) {
        return 0.0f;
    }
    if (highHz > sampleRateHz / 2) {
        highHz = sampleRateHz / 2;
    }
    if (lowHz > sampleRateHz / 2) {
        return 0.0f;
    }

    int kstart = std::ceil(static_cast<float>(lowHz) * static_cast<float>(fftSize) / static_cast<float>(sampleRateHz)); // start index
    int kend = std::floor(static_cast<float>(highHz) * static_cast<float>(fftSize) / static_cast<float>(sampleRateHz)); // end index

    if (kstart > kend) return 0.0f;

    double magnitudeSquared = 0.0;
    double BandPower = 0.0;
    for (int k = kstart; k <= kend; ++k){
        const float real = spectrumReal[k];
        const float imag = spectrumImaginary[k];
        magnitudeSquared = real * real + imag * imag;
        BandPower += magnitudeSquared;
    }

    return static_cast<float>((BandPower)/(static_cast<double>(kend - kstart + 1)));

}

float MotionScoutDetector::calculateSnrDb(const float* spectrumReal,
                                         const float* spectrumImaginary,
                                         size_t fftSize,
                                         uint32_t sampleRateHz) const {
    if (spectrumReal == nullptr || spectrumImaginary == nullptr
        || fftSize < 2 || sampleRateHz == 0) {
        return 0.0f;
    }

    float targetPower = calculateBandPower(spectrumReal, spectrumImaginary, fftSize, sampleRateHz, _config.targetLowHz, _config.targetHighHz);
    float lowerNoisePower = calculateBandPower(spectrumReal, spectrumImaginary, fftSize, sampleRateHz, _config.lowerNoiseLowHz, _config.lowerNoiseHighHz);
    float upperNoisePower = calculateBandPower(spectrumReal, spectrumImaginary, fftSize, sampleRateHz, _config.upperNoiseLowHz, _config.upperNoiseHighHz);
    float noisePower = (lowerNoisePower + upperNoisePower) / 2.0f;
    float SNRDb = 10.0f * log10f((targetPower + 1e-12f)/(noisePower + 1e-12f));
    return SNRDb;
}

MotionScoutDetector::Result MotionScoutDetector::update(
    const float* microphoneSnrDb,
    size_t microphoneCount) {
  Result result;

  if (microphoneSnrDb == nullptr ||
      microphoneCount == 0 ||
      _config.requiredMicrophones == 0 ||
      _config.requiredMicrophones > microphoneCount ||
      _config.requiredConsecutiveFrames == 0) {
    reset();
    return result;
  }

  double snrSumDb = 0.0;
  for (size_t microphone = 0; microphone < microphoneCount; ++microphone) {
    const float snrDb = microphoneSnrDb[microphone];
    if (!isfinite(snrDb)) {
      reset();
      return Result{};
    }

    snrSumDb += static_cast<double>(snrDb);
    if (snrDb >= _config.minimumSnrDb) {
      ++result.matchingMicrophones;
    }
  }

  result.averageSnrDb =
      static_cast<float>(snrSumDb / static_cast<double>(microphoneCount));

  if (result.matchingMicrophones >= _config.requiredMicrophones) {
    if (_consecutiveFrames < _config.requiredConsecutiveFrames) {
      ++_consecutiveFrames;
    }
  } else {
    _consecutiveFrames = 0;
  }

  result.detected =
      _consecutiveFrames >= _config.requiredConsecutiveFrames;
  return result;
}
