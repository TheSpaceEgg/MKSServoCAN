#pragma once
#include "ICanBus.h"
#include <SPI.h>
#include <mcp_can.h>

class McpCan : public ICanBus {
public:
  /**
   * @param miso     ESP32 GPIO connected to MCP SO
   * @param mosi     ESP32 GPIO connected to MCP SI
   * @param sclk     ESP32 GPIO connected to MCP SCK
   * @param csPin    ESP32 GPIO connected to MCP CS
   * @param intPin   ESP32 GPIO connected to MCP INT
   * @param bitrate  one of CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, etc.
   */
  McpCan(uint8_t miso,
         uint8_t mosi,
         uint8_t sclk,
         uint8_t csPin,
         uint8_t intPin,
         uint32_t bitrate = CAN_500KBPS);

  bool begin() override;
  bool send(const CanFrame &tx, uint32_t timeoutMs) override;
  bool receive(CanFrame &rx, uint32_t timeoutMs) override;

private:
  MCP_CAN _drv;
  uint8_t _miso, _mosi, _sclk, _csPin, _intPin;
  uint32_t _bitrate;
};

