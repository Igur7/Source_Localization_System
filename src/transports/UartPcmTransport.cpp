#include "transports/UartPcmTransport.h"

namespace {

constexpr uint32_t kAudioPacketMagic = 0x314D4350;
constexpr uint16_t kAudioPacketVersion = 1;
constexpr uint16_t kTaggedAudioPacketVersion = 2;
constexpr uint16_t kSyncedTaggedAudioPacketVersion = 3;

struct AudioPacketHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;
  uint32_t sequence;
  uint32_t sampleRateHz;
  uint16_t sampleCount;
  uint16_t reserved;
  uint64_t timestampUs;
} __attribute__((packed)); // wywalenie paddingu struktury, bo tak to się mogły róznić paczki

struct TaggedAudioPacketHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;
  uint32_t sequence;
  uint32_t sampleRateHz;
  uint16_t sampleCount;
  uint8_t sourceId;
  uint8_t reserved;
  uint64_t timestampUs;
} __attribute__((packed));

struct SyncedTaggedAudioPacketHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;
  uint32_t sequence;
  uint32_t sampleRateHz;
  uint16_t sampleCount;
  uint8_t sourceId;
  uint8_t reserved;
  uint32_t frameId;
  uint64_t timestampUs;
} __attribute__((packed));

}  // namespace

UartPcmTransport::UartPcmTransport()
    : UartPcmTransport(Config{}) {}

UartPcmTransport::UartPcmTransport(const Config& config)
    : _config(config) {}

UartPcmTransport::~UartPcmTransport() {
  end();
}

bool UartPcmTransport::begin() {
  if (_started) {
    return true;
  }

  if (_config.stream == nullptr || _config.baudRate == 0) {
    return false;
  }

  if (_config.hardwareSerial != nullptr) {
    if (_config.rxPin >= 0 || _config.txPin >= 0) {
      _config.hardwareSerial->begin(_config.baudRate, SERIAL_8N1, _config.rxPin, _config.txPin);
    } else {
      _config.hardwareSerial->begin(_config.baudRate);
    }
  }
  delay(100);

  _sequence = 0;
  _started = true;
  return true;
}

bool UartPcmTransport::sendPcm16(const int16_t* samples,
                                 size_t sampleCount,
                                 uint32_t sampleRateHz,
                                 uint64_t timestampUs) {
  if (!_started || samples == nullptr || sampleCount == 0 || sampleCount > UINT16_MAX) {
    return false;
  }

  const AudioPacketHeader header{
      kAudioPacketMagic,
      kAudioPacketVersion,
      static_cast<uint16_t>(sizeof(AudioPacketHeader)),
      _sequence++,
      sampleRateHz,
      static_cast<uint16_t>(sampleCount),
      0,
      timestampUs};

  const size_t headerBytes = _config.stream->write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t payloadBytes = _config.stream->write(reinterpret_cast<const uint8_t*>(samples), sampleCount * sizeof(int16_t));

  return headerBytes == sizeof(header) && payloadBytes == sampleCount * sizeof(int16_t);
}

bool UartPcmTransport::sendTaggedPcm16(uint8_t sourceId,
                                       const int16_t* samples,
                                       size_t sampleCount,
                                       uint32_t sampleRateHz,
                                       uint64_t timestampUs) {
  if (!_started || samples == nullptr || sampleCount == 0 || sampleCount > UINT16_MAX) {
    return false;
  }

  const TaggedAudioPacketHeader header{
      kAudioPacketMagic,
      kTaggedAudioPacketVersion,
      static_cast<uint16_t>(sizeof(TaggedAudioPacketHeader)),
      _sequence++,
      sampleRateHz,
      static_cast<uint16_t>(sampleCount),
      sourceId,
      0,
      timestampUs};

  const size_t headerBytes = _config.stream->write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t payloadBytes = _config.stream->write(reinterpret_cast<const uint8_t*>(samples), sampleCount * sizeof(int16_t));

  return headerBytes == sizeof(header) && payloadBytes == sampleCount * sizeof(int16_t);
}

bool UartPcmTransport::sendSyncedTaggedPcm16(uint8_t sourceId,
                                             uint32_t frameId,
                                             const int16_t* samples,
                                             size_t sampleCount,
                                             uint32_t sampleRateHz,
                                             uint64_t timestampUs) {
  if (!_started || samples == nullptr || sampleCount == 0 || sampleCount > UINT16_MAX) {
    return false;
  }

  const SyncedTaggedAudioPacketHeader header{
      kAudioPacketMagic,
      kSyncedTaggedAudioPacketVersion,
      static_cast<uint16_t>(sizeof(SyncedTaggedAudioPacketHeader)),
      _sequence++,
      sampleRateHz,
      static_cast<uint16_t>(sampleCount),
      sourceId,
      0,
      frameId,
      timestampUs};

  const size_t headerBytes = _config.stream->write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t payloadBytes = _config.stream->write(reinterpret_cast<const uint8_t*>(samples), sampleCount * sizeof(int16_t));

  return headerBytes == sizeof(header) && payloadBytes == sampleCount * sizeof(int16_t);
}

void UartPcmTransport::end() {
  if (_started && _config.stream != nullptr) {
    _config.stream->flush();
  }

  _started = false;
  _sequence = 0;
}
