#pragma once
#include "ICanBus.h"
#include "driver/twai.h"

class TwaiCan : public ICanBus {
public:
  TwaiCan(gpio_num_t txPin = GPIO_NUM_27,
          gpio_num_t rxPin = GPIO_NUM_26,
          twai_mode_t mode = TWAI_MODE_NORMAL);
  bool begin() override;
  bool send(const CanFrame& frame, uint32_t timeoutMs) override;
  bool receive(CanFrame& frame, uint32_t timeoutMs) override;
private:
  gpio_num_t _tx, _rx;
  twai_mode_t _mode;
};

