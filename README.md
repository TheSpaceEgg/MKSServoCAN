# MKSServoCAN

Arduino library for controlling **MKS SERVO42D/57D** closed‑loop stepper drivers over CAN.

---

## Features

- Full support for all official MKS CAN commands (position, speed, homing, I/O, parameters, protection, emergency stop, etc.)  
- Automatic CRC calculation and frame encoding  
- Pluggable CAN‑bus backend via the `ICanBus` interface  
  - **TWAI on ESP32** (built‑in)  
  - **SPI/MCP2515** support (tested)  
- Human‑friendly response decoding with `pollResponses()`  

---

## Installation

1. Clone into your Arduino libraries folder if not available in **Library Manager**:  
   ```bash
   cd ~/Arduino/libraries
   git clone https://github.com/YourUser/MKSServoCAN.git
   ```
2. **For ESP32/TWAI**: ensure your ESP32 board definitions are installed.  
3. **For MCP2515**: install the [MCP_CAN library](https://github.com/coryjfowler/MCP_CAN_lib) (e.g. via Library Manager) and include `<SPI.h>`.

---

## Quick Start Examples

### ESP32 + TWAI

```cpp
#include <Arduino.h>
#include <TwaiCan.h>
#include <MKSServoCAN.h>

#define MOTOR_ID  1

TwaiCan bus(GPIO_NUM_27, GPIO_NUM_26);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!MKSServoCAN::begin(&bus)) {
    Serial.println("CAN init failed");
    while (1);
  }

  MKSServoCAN::goHome(MOTOR_ID);
}

void loop() {
  MKSServoCAN::pollResponses();
}
```

### SPI + MCP2515

```cpp
#include <Arduino.h>
#include <McpCan.h>
#include <MKSServoCAN.h>

#define MOTOR_ID    1

#define MCP_MISO    19
#define MCP_MOSI    23
#define MCP_SCLK    18
#define MCP_CS      5
#define MCP_INT     17

McpCan bus(
  MCP_MISO,
  MCP_MOSI,
  MCP_SCLK,
  MCP_CS,
  MCP_INT,
  CAN_500KBPS
);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!MKSServoCAN::begin(&bus)) {
    Serial.println("CAN init failed");
    while (1);
  }

  MKSServoCAN::goHome(MOTOR_ID);
}

void loop() {
  MKSServoCAN::pollResponses();
}
```

---

## Command Reference

_All functions take a **device ID** (1–2047)._

### Status Reads

| Function                   | Code | Description                          |
|----------------------------|------|--------------------------------------|
| `readEncoderCarry(id)`     | 0x30 | 32‑bit carry + 14‑bit current value  |
| `readEncoderAdd(id)`       | 0x31 | 48‑bit cumulative encoder count      |
| `readSpeed(id)`            | 0x32 | Real‑time RPM                        |
| `readPulses(id)`           | 0x33 | 32‑bit pulse count                   |
| `readIOstatus(id)`         | 0x34 | 8‑bit I/O port status                |
| `readRawEncoder(id)`       | 0x35 | Raw 48‑bit encoder count             |
| `readAngleError(id)`       | 0x39 | Angle error (ticks)                  |
| `readEnablePin(id)`        | 0x3A | EN‑pin state                         |
| `readZeroStatus(id)`       | 0x3B | Zero‑on‑power return status          |
| `releaseProtection(id)`    | 0x3D | Release locked‑rotor protection      |
| `readProtectState(id)`     | 0x3E | Protection active (yes/no)           |

### Parameter Configuration

| Function                                | Code | Description                            |
|-----------------------------------------|------|----------------------------------------|
| `calibrate(id)`                         | 0x80 | Encoder calibration                    |
| `setWorkMode(id, mode)`                 | 0x82 | 0=CR_OPEN…5=SR_vFOC                    |
| `setCurrent(id, ma)`                    | 0x83 | Working current (mA)                   |
| `setMicrostep(id, steps)`               | 0x84 | Micro‑step subdivision                 |
| `setEnActive(id, lvl)`                  | 0x85 | EN‑pin active level                    |
| `setDirection(id, dir)`                 | 0x86 | 0=CW,1=CCW                             |
| `setAutoSleep(id, enable)`              | 0x87 | Auto turn‑off OLED                     |
| `setProtect(id, enable)`                | 0x88 | Locked‑rotor protection                |
| `setInterpolator(id, enable)`           | 0x89 | Subdivision interpolation              |
| `setHoldCurrent(id, pct)`               | 0x9B | Holding current percentage             |
| `setCanRate(id, rate)`                  | 0x8A | CAN bit rate (0=125k…3=1M)             |
| `setCanId(id, newId)`                   | 0x8B | Change device CAN ID                   |
| `setCanResponse(id, resp, act)`         | 0x8C | Slave respond & active flags           |
| `setGroupId(id, gid)`                   | 0x8D | Group address                          |
| `setKeylock(id, lock)`                  | 0x8F | Front‑panel key lock/unlock            |

### Homing & Limits

| Function                                       | Code | Description                      |
|------------------------------------------------|------|----------------------------------|
| `setHomeParams(id, trig, dir, spd, lim, mode)` | 0x90 | Configure homing                 |
| `goHome(id)`                                   | 0x91 | Execute homing                   |
| `setZeroPoint(id)`                             | 0x92 | Define current position as zero  |
| `setNoLimitReturn(id, angle, ma)`              | 0x94 | “No‑limit” return parameters     |
| `setLimitRemap(id, enable)`                    | 0x9E | Remap limit switches             |

### Utilities

| Function                                | Code     | Description                            |
|-----------------------------------------|----------|----------------------------------------|
| `setZeroMode(id, mode, en, speed, dir)` | 0x9A     | Zero on power‑up behaviour             |
| `restoreDefaults(id)`                   | 0x3F     | Reset to factory parameters            |
| `restart(id)`                           | 0x41     | Restart firmware                       |
| `setEnTrigger(id, eTrig, pProt, tim, err)` | 0x9D   | EN‑trigger zero & pos‑error prot.      |
| `readSystemParam(id, code)`             | 0x00+code| Read arbitrary system parameter        |

### Motion & Control

| Function                                    | Code         | Description                         |
|---------------------------------------------|--------------|-------------------------------------|
| `queryStatus(id)`                           | 0xF1         | Motor status                        |
| `enableMotor(id, enable)`                   | 0xF3         | Enable/disable motor                |
| `emergencyStop(id)`                         | 0xF7         | Emergency stop                      |
| `speedMode(id, speed, accel, ccw)`          | 0xF6         | Constant speed mode                 |
| `speedModeStop(id)`                         | 0xF6 (acc=0) | Stop speed mode                     |
| `speedState(id, save)`                      | 0xFF         | Save/clear speed parameters         |
| `posRelative(id, pulses, speed, accel, ccw)`| 0xFD         | Relative‑pulse move                 |
| `posRelativeStop(id)`                       | 0xFD (acc=0) | Stop relative move                  |
| `posAbsolute(id, axis, speed, accel)`       | 0xFE         | Absolute‑position move              |
| `posAbsoluteStop(id)`                       | 0xFE (acc=0) | Stop absolute move                  |
| `posAxis(id, relAxis, speed, accel)`        | 0xF4         | Relative‑axis move                  |
| `posAxisStop(id)`                           | 0xF4 (acc=0) | Stop axis move                      |

---

## Supported Hardware

- **ESP32 WROOM** with built‑in TWAI peripheral  
- **Waveshare SN65HVD230** CAN transceiver  
- **MCP2515** SPI‑to‑CAN bridge (3.3 V logic + 5 V VCC)  
- **MKS SERVO42D/57D** driver modules  

---

## Future Plans

- Support for A/B/C/D/E variants of the SERVO42/57 drivers  
- Additional SPI‑based CAN backends (e.g. on AVR, STM32, Arduino Uno)  
- Linux SBC support (Raspberry Pi, Jetson)  
- Expanded MCU compatibility (Due, Teensy, etc.)  
- Collaborative hardware compatibility matrix — **please share your test results!**

---

## External Resources

- Official MKS SERVO42C repo: https://github.com/makerbase-motor/MKS-SERVO42C  
- Official MKS SERVO57D repo: https://github.com/makerbase-motor/MKS-SERVO57D  

---

## Contributing

Pull requests and issues welcome. When submitting, please include your:

- MCU type  
- CAN transceiver model  
- Hardware wiring (pin assignments)  
- Test outcome (working/non‑working)  

---

## License

Creative Commons Attribution‑NonCommercial 4.0 International  
(CC BY‑NC 4.0)  

Anyone may copy, share, modify, and distribute this code with attribution to **Will Hickmott**.  
Commercial use (selling, bundling in a paid product, etc.) is **forbidden**.
