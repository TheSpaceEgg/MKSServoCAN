#include "McpCan.h"
#include <Arduino.h>
#include <cstring>

McpCan::McpCan(uint8_t miso,
               uint8_t mosi,
               uint8_t sclk,
               uint8_t csPin,
               uint8_t intPin,
               uint32_t bitrate)
  : _drv(csPin)
  , _miso(miso)
  , _mosi(mosi)
  , _sclk(sclk)
  , _csPin(csPin)
  , _intPin(intPin)
  , _bitrate(bitrate)
{}

bool McpCan::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  SPI.begin(_sclk, _miso, _mosi, _csPin);

  if (_drv.begin(MCP_ANY, _bitrate, MCP_8MHZ) != CAN_OK)
    return false;

  _drv.setMode(MCP_NORMAL);
  pinMode(_intPin, INPUT);
  return true;
}

bool McpCan::send(const CanFrame &tx, uint32_t /*timeoutMs*/) {
	
  Serial.print("TX \u2192 ID=0x"); 
  Serial.print(tx.id, HEX);
  Serial.print(" DLC="); 
  Serial.print(tx.dlc);
  Serial.print(" Data:");
  for (uint8_t i = 0; i < tx.dlc; i++) {
    Serial.printf(" %02X", tx.data[i]);
  }
  Serial.println();
	
	
  byte ret = _drv.sendMsgBuf(
    tx.id,        // 11-bit CAN ID
    0,            // standard frame
    tx.dlc,       // data length
    (byte*)tx.data
  );
  return (ret == CAN_OK);
}

bool McpCan::receive(CanFrame &rx, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (_drv.checkReceive() == CAN_MSGAVAIL) {
      unsigned long id;
      uint8_t len, buf[8];
      _drv.readMsgBuf(&id, &len, buf);

      rx.id  = (uint32_t)id;
      rx.dlc = len;
      memcpy(rx.data, buf, len);
      return true;
    }
  }
  return false;
}

