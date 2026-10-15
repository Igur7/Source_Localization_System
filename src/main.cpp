#include <Arduino.h>

#include "drivers/Sph0645DualMicDriver.h"
#ifdef FOUR_MIC_RECORDER
#include "services/SquareMicStreamService.h"
#include "transports/UartPcmTransport.h"
#else
#include "services/SquareSoundLocalizationService.h"
#endif

namespace {
#ifdef FOUR_MIC_RECORDER
constexpr uint32_t kSerialBaudRate = 2000000;
constexpr uint32_t kSampleRateHz = 16000;
#else
constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kSampleRateHz = 48000;
#endif
constexpr size_t kChunkSamples = 512;
constexpr float kArrayWidthMeters = 0.134f;   // lewo-prawo, między portami
constexpr float kArrayHeightMeters = 0.140f;  // góra-dół, między portami

// Przekątna A: top-left (LEFT) oraz bottom-right (RIGHT).
constexpr int kDiagonalABclkPin = 46;
constexpr int kDiagonalAWsPin = 10;
constexpr int kDiagonalADataPin = 12;

// Przekątna B: top-right (LEFT) oraz bottom-left (RIGHT).
constexpr int kDiagonalBBclkPin = 1;
constexpr int kDiagonalBWsPin = 42;
constexpr int kDiagonalBDataPin = 40;
constexpr uint32_t kMaxLocalizationSeconds = 300;

Sph0645DualMicDriver::Config makeDiagonalAMicsConfig() {
  Sph0645DualMicDriver::Config config;
  config.port = I2S_NUM_1;
  config.bclkPin = kDiagonalABclkPin;
  config.wsPin = kDiagonalAWsPin;
  config.dataInPin = kDiagonalADataPin;
  config.masterClock = true;
  config.sampleRateHz = kSampleRateHz;
  config.dmaBufferCount = 16;
  config.dmaBufferLength = 256;
  config.readFrameCount = kChunkSamples;
  config.removeDcOffset = true;
  return config;
}

Sph0645DualMicDriver::Config makeDiagonalBMicsConfig() {
  Sph0645DualMicDriver::Config config;
  config.port = I2S_NUM_0;
  config.bclkPin = kDiagonalBBclkPin;
  config.wsPin = kDiagonalBWsPin;
  config.dataInPin = kDiagonalBDataPin;
  config.masterClock = true;
  config.sampleRateHz = kSampleRateHz;
  config.dmaBufferCount = 16;
  config.dmaBufferLength = 256;
  config.readFrameCount = kChunkSamples;
  config.removeDcOffset = true;
  return config;
}

#ifdef FOUR_MIC_RECORDER
UartPcmTransport::Config makeTransportConfig() {
  UartPcmTransport::Config config;
  config.stream = &Serial;
  config.baudRate = kSerialBaudRate;
  return config;
}
#else
SquareSoundLocalizationService::Config makeLocalizationConfig() {
  SquareSoundLocalizationService::Config config;
  config.sampleRateHz = kSampleRateHz;
  config.chunkSamples = kChunkSamples;
  config.arrayWidthMeters = kArrayWidthMeters;
  config.arrayHeightMeters = kArrayHeightMeters;
  config.speedOfSoundMetersPerSecond = 343.0f;
  config.minimumRms = 80.0f;
  config.minimumPairPeakRatio = 1.5f;
  config.maximumResidualMeters = 0.03f;
  config.smoothingWindow = 5;
  config.reportIntervalMs = 200;
  #ifdef MOTION_SCOUT_DETECTOR
    config.motionScoutDetectionEnabled = true;
  #endif
    return config;
}
#endif

Sph0645DualMicDriver diagonalAMics(makeDiagonalAMicsConfig());
Sph0645DualMicDriver diagonalBMics(makeDiagonalBMicsConfig());
#ifdef FOUR_MIC_RECORDER
UartPcmTransport transport(makeTransportConfig());
SquareMicStreamService activeService(
    diagonalAMics, diagonalBMics, transport, kSampleRateHz, kChunkSamples);
#else
SquareSoundLocalizationService activeService(
    diagonalAMics, diagonalBMics, makeLocalizationConfig());
#endif
String commandBuffer;
uint32_t serviceStopAtMs = 0;

uint32_t parseDurationSeconds(const String& command) {
  const int separator = command.indexOf(' ');
  if (separator < 0) {
    return 0;
  }

  const uint32_t durationSeconds = static_cast<uint32_t>(command.substring(separator + 1).toInt());
  if (durationSeconds > kMaxLocalizationSeconds) {
    return kMaxLocalizationSeconds;
  }

  return durationSeconds;
}

void stopLocalization(const char* reason) {
  if (activeService.isRunning()) {
    activeService.stop();
  }

  serviceStopAtMs = 0;
  Serial.print("OK STOP ");
  Serial.println(reason);
}

void startLocalization(uint32_t durationSeconds) {
  if (durationSeconds == 0) {
    Serial.println("ERR duration_seconds_required");
    return;
  }

  if (activeService.isRunning()) {
    activeService.stop();
  }

  Serial.print("OK START ");
  Serial.println(durationSeconds);
  Serial.flush();
  delay(20);

  serviceStopAtMs = millis() + durationSeconds * 1000UL;
  if (!activeService.start()) {
    serviceStopAtMs = 0;
    Serial.println("ERR service_start_failed");
    return;
  }
}

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.startsWith("START ")) {
    startLocalization(parseDurationSeconds(command));
    return;
  }

  if (command == "STOP") {
    stopLocalization("REQUEST");
    return;
  }

  if (command == "STATUS") {
    Serial.println(activeService.isRunning() ? "OK RUNNING" : "OK IDLE");
    return;
  }

  if (command.length() > 0) {
    Serial.println("ERR unknown_command");
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      handleCommand(commandBuffer);
      commandBuffer = "";
      continue;
    }

    if (commandBuffer.length() < 64) {
      commandBuffer += c;
    } else {
      commandBuffer = "";
      Serial.println("ERR command_too_long");
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaudRate);
  delay(1000);

  Serial.println("OK READY use START <seconds>");
}

void loop() {
  readCommands();

  if (serviceStopAtMs != 0 &&
      static_cast<int32_t>(millis() - serviceStopAtMs) >= 0) {
    stopLocalization("TIMEOUT");
  }

#ifndef FOUR_MIC_RECORDER
  float angleDegrees = 0.0f;
  if (activeService.takeLatestAngle(angleDegrees)) {
    Serial.print("ANGLE ");
    Serial.println(angleDegrees, 2);
  }
#endif

  delay(5);
}
