#pragma once

#include "core/FreeRtosService.h"
#include "drivers/Sph0645DualMicDriver.h"
#include "transports/IAudioTransport.h"

class SquareMicStreamService : public FreeRtosService {
public:
  SquareMicStreamService(Sph0645DualMicDriver& diagonalAMics,
                         Sph0645DualMicDriver& diagonalBMics,
                         IAudioTransport& transport,
                         uint32_t sampleRateHz,
                         size_t chunkSamples);

protected:
  bool onStart() override;
  void onLoop() override;
  void onStop() override;

private:
  Sph0645DualMicDriver& _diagonalAMics;
  Sph0645DualMicDriver& _diagonalBMics;
  IAudioTransport& _transport;
  uint32_t _sampleRateHz;
  size_t _chunkSamples;
  uint32_t _frameId = 0;
  int16_t* _audio[4] = {};
};
