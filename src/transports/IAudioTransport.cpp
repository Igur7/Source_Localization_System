#include "transports/IAudioTransport.h"

bool IAudioTransport::sendTaggedPcm16(uint8_t,
                                      const int16_t* samples,
                                      size_t sampleCount,
                                      uint32_t sampleRateHz,
                                      uint64_t timestampUs) {
  return sendPcm16(samples, sampleCount, sampleRateHz, timestampUs);
}

bool IAudioTransport::sendSyncedTaggedPcm16(uint8_t sourceId,
                                            uint32_t,
                                            const int16_t* samples,
                                            size_t sampleCount,
                                            uint32_t sampleRateHz,
                                            uint64_t timestampUs) {
  return sendTaggedPcm16(sourceId, samples, sampleCount, sampleRateHz, timestampUs);
}
