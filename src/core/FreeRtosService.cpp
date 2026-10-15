#include "core/FreeRtosService.h"

FreeRtosService::FreeRtosService(const char* name,
                                 uint32_t stackSize,
                                 UBaseType_t priority,
                                 BaseType_t coreId)
    : _name(name),
      _stackSize(stackSize),
      _priority(priority),
      _coreId(coreId) {}

FreeRtosService::~FreeRtosService() {
  stop();
}

bool FreeRtosService::start() {
  if (_taskHandle != nullptr) {
    return false;
  }

  _stopRequested = false;
  _running = false;

  BaseType_t result = xTaskCreatePinnedToCore(
      taskEntryPoint,
      _name,
      _stackSize,
      this,
      _priority,
      &_taskHandle,
      _coreId);

  return result == pdPASS;
}

void FreeRtosService::requestStop() {
  _stopRequested = true;
}

void FreeRtosService::stop() {
  if (_taskHandle == nullptr) {
    _running = false;
    return;
  }

  requestStop();

  while (_taskHandle != nullptr) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

bool FreeRtosService::isRunning() const {
  return _running;
}

const char* FreeRtosService::getName() const {
  return _name;
}

bool FreeRtosService::shouldStop() const {
  return _stopRequested;
}

void FreeRtosService::sleepMs(uint32_t ms) const {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

void FreeRtosService::taskEntryPoint(void* parameter) {
  FreeRtosService* service = static_cast<FreeRtosService*>(parameter);
  service->run();
}

void FreeRtosService::run() {
  _running = true;

  if (!onStart()) {
    _running = false;
    _taskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  while (!_stopRequested) {
    onLoop();
  }

  onStop();

  _running = false;
  _taskHandle = nullptr;
  vTaskDelete(nullptr);
}