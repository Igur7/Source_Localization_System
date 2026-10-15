#pragma once

class IService {
public:
  virtual ~IService() = default;

  virtual bool start() = 0;
  virtual void requestStop() = 0;
  virtual void stop() = 0;
  virtual bool isRunning() const = 0;
  virtual const char* getName() const = 0;
};