#include "drivers/Sph0645MicDriver.h"

#include <limits.h>
#include <stdlib.h>

Sph0645MicDriver::Sph0645MicDriver()
    : Sph0645MicDriver(Config{}) {}

Sph0645MicDriver::Sph0645MicDriver(const Config& config)
    : _config(config) {}

Sph0645MicDriver::~Sph0645MicDriver() {
  end();
}

bool Sph0645MicDriver::begin() {
  if (_started) {
    return true;
  }

  if (_config.readFrameCount == 0 || _config.dmaBufferCount == 0 || _config.dmaBufferLength == 0) {
    return false;
  }

  _rawWordCapacity = _config.readFrameCount * 2;
  _rawBuffer = static_cast<int32_t*>(malloc(_rawWordCapacity * sizeof(int32_t)));
  if (_rawBuffer == nullptr) {
    return false;
  }

  i2s_config_t i2sConfig = {};
  i2sConfig.mode = static_cast<i2s_mode_t>((_config.masterClock ? I2S_MODE_MASTER : I2S_MODE_SLAVE) | I2S_MODE_RX);
  i2sConfig.sample_rate = static_cast<int>(_config.sampleRateHz);
  i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  i2sConfig.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2sConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2sConfig.dma_buf_count = static_cast<int>(_config.dmaBufferCount);
  i2sConfig.dma_buf_len = static_cast<int>(_config.dmaBufferLength);
  i2sConfig.use_apll = false;
  i2sConfig.tx_desc_auto_clear = false;
  i2sConfig.fixed_mclk = 0;
  i2sConfig.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT;
  i2sConfig.bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT;

  esp_err_t err = i2s_driver_install(_config.port, &i2sConfig, 0, nullptr);
  if (err != ESP_OK) {
    end();
    return false;
  }

  i2s_pin_config_t pinConfig = {};
  pinConfig.bck_io_num = _config.bclkPin;
  pinConfig.ws_io_num = _config.wsPin;
  pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
  pinConfig.data_in_num = _config.dataInPin;

  err = i2s_set_pin(_config.port, &pinConfig);
  if (err != ESP_OK) {
    end();
    return false;
  }

  i2s_zero_dma_buffer(_config.port);
  _lastDcOffset = 0;
  _started = true;
  return true;
}

bool Sph0645MicDriver::read(int16_t* buffer, size_t capacity, size_t& outSamples) {
  outSamples = 0;

  if (!_started || buffer == nullptr || capacity == 0) {
    return false;
  }

  const size_t framesToRead = (capacity < _config.readFrameCount) ? capacity : _config.readFrameCount;
  const size_t bytesToRead = framesToRead * 2 * sizeof(int32_t);

  size_t bytesRead = 0;
  const esp_err_t err = i2s_read(_config.port, _rawBuffer, bytesToRead, &bytesRead, pdMS_TO_TICKS(1000));
  if (err != ESP_OK || bytesRead == 0) {
    return false;
  }

  const size_t wordsRead = bytesRead / sizeof(int32_t);
  const size_t slotIndex = (_config.channel == Channel::Left) ? 0U : 1U;

  for (size_t i = 0; i + 1 < wordsRead && outSamples < capacity; i += 2) {
    buffer[outSamples++] = rawWordToPcm16(_rawBuffer[i + slotIndex]);
  }

  if (outSamples == 0) {
    return false;
  }

  _lastDcOffset = computeMean(buffer, outSamples);

  if (_config.removeDcOffset) {
    removeDcOffsetFromBuffer(buffer, outSamples, _lastDcOffset);
  }

  return true;
}

void Sph0645MicDriver::end() {
  if (_started) {
    i2s_driver_uninstall(_config.port);
    _started = false;
  }

  if (_rawBuffer != nullptr) {
    free(_rawBuffer);
    _rawBuffer = nullptr;
  }

  _rawWordCapacity = 0;
  _lastDcOffset = 0;
}

int32_t Sph0645MicDriver::lastDcOffset() const {
  return _lastDcOffset;
}

uint32_t Sph0645MicDriver::sampleRateHz() const {
  return _config.sampleRateHz;
}

int16_t Sph0645MicDriver::rawWordToPcm16(int32_t rawWord) {
  int32_t sample = rawWord >> kRawShift;
  sample >>= kPcmShift;

  if (sample > INT16_MAX) {
    sample = INT16_MAX;
  } else if (sample < INT16_MIN) {
    sample = INT16_MIN;
  }

  return static_cast<int16_t>(sample);
}

int32_t Sph0645MicDriver::computeMean(const int16_t* buffer, size_t count) {
  if (buffer == nullptr || count == 0) {
    return 0;
  }

  int64_t sum = 0;
  for (size_t i = 0; i < count; ++i) {
    sum += buffer[i];
  }

  return static_cast<int32_t>(sum / static_cast<int64_t>(count));
}

void Sph0645MicDriver::removeDcOffsetFromBuffer(int16_t* buffer, size_t count, int32_t mean) {
  if (buffer == nullptr || count == 0) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    int32_t centered = static_cast<int32_t>(buffer[i]) - mean;

    if (centered > INT16_MAX) {
      centered = INT16_MAX;
    } else if (centered < INT16_MIN) {
      centered = INT16_MIN;
    }

    buffer[i] = static_cast<int16_t>(centered);
  }
}
