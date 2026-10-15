#include "services/SquareMicStreamService.h"

#include <esp_timer.h>
#include <stdlib.h>

namespace {
constexpr uint8_t kTopLeftSourceId = 1;
constexpr uint8_t kTopRightSourceId = 2;
constexpr uint8_t kBottomLeftSourceId = 3;
constexpr uint8_t kBottomRightSourceId = 4;
}  // namespace

SquareMicStreamService::SquareMicStreamService(Sph0645DualMicDriver& diagonalAMics,
                                               Sph0645DualMicDriver& diagonalBMics,
                                               IAudioTransport& transport,
                                               uint32_t sampleRateHz,
                                               size_t chunkSamples)
    : FreeRtosService("SquareMicRecorder", 8192, 2, 1),
      _diagonalAMics(diagonalAMics),
      _diagonalBMics(diagonalBMics),
      _transport(transport),
      _sampleRateHz(sampleRateHz),
      _chunkSamples(chunkSamples) {}

bool SquareMicStreamService::onStart() {
  if (_chunkSamples == 0) {
    return false;
  }

  for (size_t microphone = 0; microphone < 4; ++microphone) {
    _audio[microphone] =
        static_cast<int16_t*>(malloc(_chunkSamples * sizeof(int16_t)));
    if (_audio[microphone] == nullptr) {
      onStop();
      return false;
    }
  }

  if (!_diagonalAMics.begin() || !_diagonalBMics.begin() || !_transport.begin()) {
    onStop();
    return false;
  }

  _frameId = 0;
  return true;
}

void SquareMicStreamService::onLoop() {
  size_t diagonalASamples = 0;
  size_t diagonalBSamples = 0;
  const bool diagonalAOk = _diagonalAMics.read(
      _audio[0], _audio[3], _chunkSamples, diagonalASamples);
  const bool diagonalBOk = _diagonalBMics.read(
      _audio[1], _audio[2], _chunkSamples, diagonalBSamples);

  if (!diagonalAOk || !diagonalBOk ||
      diagonalASamples == 0 || diagonalBSamples == 0) {
    sleepMs(2);
    return;
  }

  const size_t sampleCount =
      (diagonalASamples < diagonalBSamples)
          ? diagonalASamples
          : diagonalBSamples;
  const uint32_t frameId = _frameId++;
  const uint64_t timestampUs = static_cast<uint64_t>(esp_timer_get_time());

  _transport.sendSyncedTaggedPcm16(
      kTopLeftSourceId, frameId, _audio[0], sampleCount, _sampleRateHz, timestampUs);
  _transport.sendSyncedTaggedPcm16(
      kTopRightSourceId, frameId, _audio[1], sampleCount, _sampleRateHz, timestampUs);
  _transport.sendSyncedTaggedPcm16(
      kBottomLeftSourceId, frameId, _audio[2], sampleCount, _sampleRateHz, timestampUs);
  _transport.sendSyncedTaggedPcm16(
      kBottomRightSourceId, frameId, _audio[3], sampleCount, _sampleRateHz, timestampUs);
}

void SquareMicStreamService::onStop() {
  _transport.end();
  _diagonalBMics.end();
  _diagonalAMics.end();

  for (size_t microphone = 0; microphone < 4; ++microphone) {
    free(_audio[microphone]);
    _audio[microphone] = nullptr;
  }
}
