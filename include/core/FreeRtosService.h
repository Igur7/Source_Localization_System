#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "core/IService.h"

class FreeRtosService : public IService {
public:
  FreeRtosService(const char* name,
                  uint32_t stackSize = 4096,
                  UBaseType_t priority = 1,
                  BaseType_t coreId = tskNO_AFFINITY);
  virtual ~FreeRtosService();

  bool start() override;
  void requestStop() override;
  void stop() override;
  bool isRunning() const override;
  const char* getName() const override;

protected:
  virtual bool onStart() = 0;
  virtual void onLoop() = 0;
  virtual void onStop() = 0;

  bool shouldStop() const;
  void sleepMs(uint32_t ms) const;

private:
  static void taskEntryPoint(void* parameter);
  void run();

  const char* _name;
  uint32_t _stackSize;
  UBaseType_t _priority;
  BaseType_t _coreId;
  volatile bool _running = false;
  volatile bool _stopRequested = false;
  TaskHandle_t _taskHandle = nullptr;
};