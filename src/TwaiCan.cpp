#include "TwaiCan.h"
#include <Arduino.h>
#include <string.h>

TwaiCan::TwaiCan(gpio_num_t txPin, gpio_num_t rxPin, twai_mode_t mode)
  : _tx(txPin), _rx(rxPin), _mode(mode) {}

bool TwaiCan::begin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(_tx, _rx, _mode);
  twai_timing_config_t   t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t   f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return (twai_start() == ESP_OK);
}

bool TwaiCan::send(const CanFrame& in, uint32_t timeoutMs) {
  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier       = in.id;              
  msg.extd             = 0;                 
  msg.data_length_code = in.dlc;             
  memcpy(msg.data, in.data, in.dlc);         

  Serial.print("TX \u2192 ID=0x"); 
  Serial.print(in.id, HEX);
  Serial.print(" DLC="); 
  Serial.print(in.dlc);
  Serial.print(" Data:");
  for (uint8_t i = 0; i < in.dlc; i++) {
    Serial.printf(" %02X", in.data[i]);
  }
  Serial.println();

  return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}


bool TwaiCan::receive(CanFrame& out, uint32_t timeoutMs) {
  twai_message_t msg;
  while (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK) {
    if (msg.flags & TWAI_MSG_FLAG_SELF) continue;

    out.id  = msg.identifier;
    out.dlc = msg.data_length_code;
    memcpy(out.data, msg.data, msg.data_length_code);
    return true;
  }
  return false;
}

