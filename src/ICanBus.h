#pragma once
#include <stdint.h>

struct CanFrame {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

class ICanBus {
public:
  virtual ~ICanBus() {}
  virtual bool begin() = 0;
  virtual bool send(const CanFrame& frame, uint32_t timeoutMs) = 0;
  virtual bool receive(CanFrame& frame, uint32_t timeoutMs) = 0;
};

