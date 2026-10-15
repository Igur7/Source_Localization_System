#pragma once

#include <Arduino.h>

class IMicDriver {
public:
  virtual ~IMicDriver() = default;

  virtual bool begin() = 0;
  virtual bool read(int16_t* buffer, size_t capacity, size_t& outSamples) = 0;
  virtual void end() = 0;
};