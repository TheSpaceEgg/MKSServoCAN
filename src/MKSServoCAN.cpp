// File: libraries/MKSServoCAN/src/MKSServoCAN.cpp

#include "MKSServoCAN.h"
#include "Arduino.h"
#include <vector>
#include <cstring>

namespace MKSServoCAN {

/// Pointer to the underlying CAN bus implementation
static ICanBus* _bus = nullptr;

/// @brief Compute the 8-bit CRC for a CAN frame payload.
/// @param id    CAN identifier (added into CRC sum)
/// @param data  Pointer to payload bytes
/// @param n     Number of payload bytes
/// @return      8-bit checksum = (id + sum(data[0..n-1])) & 0xFF
static uint8_t computeCRC(uint32_t id, const uint8_t* data, size_t n) {
  uint32_t sum = id;
  for (size_t i = 0; i < n; ++i) {
    sum += data[i];
  }
  return uint8_t(sum & 0xFF);
}

/// @brief Append CRC to payload and send as a CAN frame.
/// @param id        CAN identifier to send to
/// @param payload   Vector of payload bytes (without CRC)
static void sendFrame(uint32_t id, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> buf = payload;
  buf.push_back(computeCRC(id, buf.data(), buf.size()));

  CanFrame frame{ id, uint8_t(buf.size()) };
  memcpy(frame.data, buf.data(), buf.size());
  _bus->send(frame, 100);
}

bool begin(ICanBus* bus) {
  _bus = bus;
  return _bus && _bus->begin();
}

/// @brief Poll and decode all pending CAN responses.
/// @param timeoutMs  Milliseconds to wait for each receive call
void pollResponses(uint32_t timeoutMs) {
  CanFrame rx;
  while (_bus->receive(rx, timeoutMs)) {
    uint8_t code = rx.data[0];
    Serial.printf("RX -> ID=0x%03X  Code=0x%02X: ", rx.id, code);

    switch (code) {
      case 0x30: {
        if (rx.dlc >= 8) {
          int32_t carry = int32_t((uint32_t)rx.data[1] << 24 |
                                  (uint32_t)rx.data[2] << 16 |
                                  (uint32_t)rx.data[3] <<  8 |
                                                   rx.data[4]);
          uint16_t val = uint16_t(rx.data[5] << 8 | rx.data[6]);
          Serial.printf("EncCarry=%ld, value=%u\n", carry, val);
        }
        break;
      }
      case 0x31: {
        if (rx.dlc >= 8) {
          int64_t v = 0;
          for (int i = 1; i <= 6; ++i) {
            v = (v << 8) | rx.data[i];
          }
          if (v & (int64_t(1) << 47)) {
            v |= ~((int64_t(1) << 48) - 1);
          }
          Serial.printf("EncAdd=%lld\n", v);
        }
        break;
      }
      case 0x32: {
        if (rx.dlc >= 4) {
          int16_t sp = int16_t(rx.data[1] << 8 | rx.data[2]);
          Serial.printf("Speed=%d RPM\n", sp);
        }
        break;
      }
      case 0x33: {
        if (rx.dlc >= 6) {
          int32_t p = int32_t((uint32_t)rx.data[1] << 24 |
                              (uint32_t)rx.data[2] << 16 |
                              (uint32_t)rx.data[3] <<  8 |
                                               rx.data[4]);
          Serial.printf("Pulses=%ld\n", p);
        }
        break;
      }
      case 0x34: {
        if (rx.dlc >= 2) {
          Serial.printf("IOstat=0b%08b\n", rx.data[1]);
        }
        break;
      }
      case 0x35: {
        if (rx.dlc >= 8) {
          int64_t v = 0;
          for (int i = 1; i <= 6; ++i) {
            v = (v << 8) | rx.data[i];
          }
          if (v & (int64_t(1) << 47)) {
            v |= ~((int64_t(1) << 48) - 1);
          }
          Serial.printf("RawEnc=%lld\n", v);
        }
        break;
      }
      case 0x39: {
        if (rx.dlc >= 6) {
          int32_t err = int32_t((uint32_t)rx.data[1] << 24 |
                                 (uint32_t)rx.data[2] << 16 |
                                 (uint32_t)rx.data[3] <<  8 |
                                                  rx.data[4]);
          float deg = err * 360.0f / 51200.0f;
          Serial.printf("ErrTicks=%ld (%.2f°)\n", err, deg);
        }
        break;
      }
      case 0x3A: {
        if (rx.dlc >= 2) {
          Serial.printf("EN=%s\n", rx.data[1] ? "Enabled" : "Disabled");
        }
        break;
      }
      case 0x3B: {
        if (rx.dlc >= 2) {
          const char* status[] = { "Going", "Success", "Fail" };
          uint8_t st = rx.data[1];
          Serial.printf("ZeroStatus=%s\n", st < 3 ? status[st] : "Unknown");
        }
        break;
      }
      case 0x3D: {
        if (rx.dlc >= 2) {
          Serial.printf("ReleaseProt=%s\n", rx.data[1] ? "OK" : "Fail");
        }
        break;
      }
      case 0x3E: {
        if (rx.dlc >= 2) {
          Serial.printf("Protected=%s\n", rx.data[1] ? "Yes" : "No");
        }
        break;
      }
      case 0x80: case 0x82: case 0x83: case 0x84:
      case 0x85: case 0x86: case 0x87: case 0x88:
      case 0x89: case 0x9B:
      case 0x8A: case 0x8B: case 0x8C: case 0x8D: case 0x8F: {
        if (rx.dlc >= 2) {
          Serial.printf("Cmd0x%02X status=%s\n",
                        code, rx.data[1] ? "OK" : "Fail");
        }
        break;
      }
      case 0x90: case 0x92: case 0x94: case 0x9E: {
        if (rx.dlc >= 2) {
          Serial.printf("HomeCmd0x%02X status=%s\n",
                        code, rx.data[1] ? "OK" : "Fail");
        }
        break;
      }
      case 0x91: {
        if (rx.dlc >= 2) {
          const char* stage[] = { "Fail", "Start", "Success" };
          uint8_t st = rx.data[1];
          Serial.printf("GoHome=%s\n", st < 3 ? stage[st] : "Unknown");
        }
        break;
      }
      case 0x9A: case 0x3F: case 0x41: {
        if (rx.dlc >= 2) {
          Serial.printf("Cmd0x%02X status=%s\n",
                        code, rx.data[1] ? "OK" : "Fail");
        }
        break;
      }
      case 0x9D: {
        if (rx.dlc >= 2) {
          Serial.printf("EnTrig/PosErr status=%s\n",
                        rx.data[1] ? "OK" : "Fail");
        }
        break;
      }
      case 0x00: {
        if (rx.dlc >= 2) {
          Serial.print("SysParam0x");
          Serial.print(rx.data[1], HEX);
          Serial.print(" =");
          for (int i = 2; i < rx.dlc; ++i) {
            Serial.printf(" %02X", rx.data[i]);
          }
          Serial.println();
        }
        break;
      }
      case 0xF1: {
        if (rx.dlc >= 2) {
          const char* stat[] = {
            "Fail","Stop","SpeedUp","SpeedDown",
            "Full","Homing","Calibrating"
          };
          uint8_t st = rx.data[1];
          Serial.printf("Status=%s\n", st < 7 ? stat[st] : "Unknown");
        }
        break;
      }
      case 0xF3: case 0xF7: case 0xFF: {
        if (rx.dlc >= 2) {
          Serial.printf("Cmd0x%02X status=%s\n",
                        code, rx.data[1] ? "OK" : "Fail");
        }
        break;
      }
      case 0xF6: {
        if (rx.dlc >= 2) {
          const char* modes[] = {
            "Fail","Running","StopStart","StopOK"
          };
          uint8_t st = rx.data[1];
          Serial.printf("SpeedMode=%s\n", st < 4 ? modes[st] : "Unknown");
        }
        break;
      }
      case 0xFD: {
        if (rx.dlc >= 2) {
          const char* rel[] = {
            "Run fail","Run starting","Run complete","End-stop"
          };
          uint8_t st = rx.data[1];
          Serial.printf("PosRel=%s\n", st < 4 ? rel[st] : "Unknown");
        }
        break;
      }
      case 0xFE: {
        if (rx.dlc >= 2) {
          const char* abs[] = {
            "Fail","Start","Complete","End-limit"
          };
          uint8_t st = rx.data[1];
          Serial.printf("PosAbs=%s\n", st < 4 ? abs[st] : "Unknown");
        }
        break;
      }
      case 0xF4: {
        if (rx.dlc >= 2) {
          const char* ax[] = {
            "Fail","Start","Complete","End-limit"
          };
          uint8_t st = rx.data[1];
          Serial.printf("PosAxis=%s\n", st < 4 ? ax[st] : "Unknown");
        }
        break;
      }
      default: {
        Serial.print("RAW:");
        for (int i = 0; i < rx.dlc; ++i) {
          Serial.printf(" %02X", rx.data[i]);
        }
        Serial.println();
      }
    }
  }
}

// read status commands
void readEncoderCarry(uint32_t id)   { sendFrame(id,{0x30}); }
void readEncoderAdd(uint32_t id)     { sendFrame(id,{0x31}); }
void readSpeed(uint32_t id)          { sendFrame(id,{0x32}); }
void readPulses(uint32_t id)         { sendFrame(id,{0x33}); }
void readIOstatus(uint32_t id)       { sendFrame(id,{0x34}); }
void readRawEncoder(uint32_t id)     { sendFrame(id,{0x35}); }
void readAngleError(uint32_t id)     { sendFrame(id,{0x39}); }
void readEnablePin(uint32_t id)      { sendFrame(id,{0x3A}); }
void readZeroStatus(uint32_t id)     { sendFrame(id,{0x3B}); }
void releaseProtection(uint32_t id)  { sendFrame(id,{0x3D}); }
void readProtectState(uint32_t id)   { sendFrame(id,{0x3E}); }

// system parameter commands
void calibrate(uint32_t id)                       { sendFrame(id,{0x80,0x00}); }
void setWorkMode(uint32_t id,uint8_t m)           { sendFrame(id,{0x82,m}); }
void setCurrent(uint32_t id,uint16_t ma)          { sendFrame(id,{0x83,uint8_t(ma>>8),uint8_t(ma)}); }
void setMicrostep(uint32_t id,uint8_t ms)         { sendFrame(id,{0x84,ms}); }
void setEnActive(uint32_t id,uint8_t en)          { sendFrame(id,{0x85,en}); }
void setDirection(uint32_t id,uint8_t dir)        { sendFrame(id,{0x86,dir}); }
void setAutoSleep(uint32_t id,bool e)             { sendFrame(id,{0x87,uint8_t(e)}); }
void setProtect(uint32_t id,bool e)               { sendFrame(id,{0x88,uint8_t(e)}); }
void setInterpolator(uint32_t id,bool e)          { sendFrame(id,{0x89,uint8_t(e)}); }
void setHoldCurrent(uint32_t id,uint8_t pct)      { sendFrame(id,{0x9B,pct}); }
void setCanRate(uint32_t id,uint8_t r)            { sendFrame(id,{0x8A,r}); }
void setCanId(uint32_t id,uint16_t nid)           { sendFrame(id,{0x8B,uint8_t(nid>>8),uint8_t(nid)}); }
void setCanResponse(uint32_t id,bool rsp,bool act){ sendFrame(id,{0x8C,uint8_t(rsp),uint8_t(act)}); }
void setGroupId(uint32_t id,uint16_t gid)         { sendFrame(id,{0x8D,uint8_t(gid>>8),uint8_t(gid)}); }
void setKeylock(uint32_t id,bool l)               { sendFrame(id,{0x8F,uint8_t(l)}); }

void setHomeParams(uint32_t id,uint8_t t,uint8_t d,uint16_t s,bool el,uint8_t m) {
  sendFrame(id,{0x90,t,d,uint8_t(s>>8),uint8_t(s),uint8_t(el),m});
}
void goHome(uint32_t id)                          { sendFrame(id,{0x91}); }
void setZeroPoint(uint32_t id)                    { sendFrame(id,{0x92}); }
void setNoLimitReturn(uint32_t id,uint32_t r,uint16_t ma) {
  sendFrame(id,{0x94,uint8_t(r>>24),uint8_t(r>>16),uint8_t(r>>8),uint8_t(r),uint8_t(ma>>8),uint8_t(ma)});
}
void setLimitRemap(uint32_t id,bool e)            { sendFrame(id,{0x9E,uint8_t(e)}); }

void setZeroMode(uint32_t id,uint8_t m,bool en,uint8_t sp,uint8_t dir) {
  sendFrame(id,{0x9A,m,uint8_t(en),sp,dir});
}

void restoreDefaults(uint32_t id)                 { sendFrame(id,{0x3F}); }
void restart(uint32_t id)                         { sendFrame(id,{0x41}); }

void setEnTrigger(uint32_t id,bool et,bool pp,uint16_t tim,uint16_t err) {
  sendFrame(id,{0x9D,uint8_t((pp?0x02:0)|(et?0x01:0)),uint8_t(tim>>8),uint8_t(tim),uint8_t(err>>8),uint8_t(err)});
}

void readSystemParam(uint32_t id,uint8_t c)       { sendFrame(id,{0x00,c}); }

void queryStatus(uint32_t id)                     { sendFrame(id,{0xF1}); }
void enableMotor(uint32_t id,bool en)             { sendFrame(id,{0xF3,uint8_t(en)}); }

void emergencyStop(uint32_t id)                   { sendFrame(id,{0xF7}); }

void speedMode(uint32_t id,uint16_t s,uint8_t a,bool ccw) {
  sendFrame(id,{0xF6,uint8_t((ccw?0x80:0)|((s>>8)&0x0F)),uint8_t(s&0xFF),a});
}
void speedModeStop(uint32_t id)                   { sendFrame(id,{0xF6,0,0,0}); }
void speedState(uint32_t id,bool save)            { sendFrame(id,{0xFF,uint8_t(save?0xC8:0xCA)}); }

void posRelative(uint32_t id,uint32_t p,uint16_t s,uint8_t a,bool ccw) {
  sendFrame(id,{0xFD,uint8_t((ccw?0x80:0)|((s>>8)&0x0F)),uint8_t(s&0xFF),a,uint8_t(p>>16),uint8_t(p>>8),uint8_t(p)});
}
void posRelativeStop(uint32_t id)                 { sendFrame(id,{0xFD,0,0,0,0,0,0}); }

void posAbsolute(uint32_t id,int32_t ax,uint16_t s,uint8_t a) {
  sendFrame(id,{0xFE,uint8_t(s>>8),uint8_t(s),a,uint8_t(ax>>16),uint8_t(ax>>8),uint8_t(ax)});
}
void posAbsoluteStop(uint32_t id)                 { sendFrame(id,{0xFE,0,0,0,0,0,0}); }

void posAxis(uint32_t id,int32_t r,uint16_t s,uint8_t a) {
  sendFrame(id,{0xF4,uint8_t(s>>8),uint8_t(s),a,uint8_t(r>>16),uint8_t(r>>8),uint8_t(r)});
}
void posAxisStop(uint32_t id)                     { sendFrame(id,{0xF4,0,0,0,0,0,0}); }

} // namespace MKSServoCAN

