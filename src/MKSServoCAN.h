#pragma once

#include "ICanBus.h"
#include <cstdint>

namespace MKSServoCAN {

/// @brief Initialise the MKSServoCAN library with a CAN bus implementation.
bool begin(ICanBus* bus);

/// @brief Poll for incoming CAN frames and decode any responses. Call periodically in your loop.
void pollResponses(uint32_t timeoutMs = 10);

/// @brief Read the 32-bit encoder carry value plus 14-bit current value (0x30).
void readEncoderCarry(uint32_t id);

/// @brief Read the cumulative encoder value (0x31).
void readEncoderAdd(uint32_t id);

/// @brief Read the real-time motor speed in RPM (0x32).
void readSpeed(uint32_t id);

/// @brief Read the number of pulses received by the driver (0x33).
void readPulses(uint32_t id);

/// @brief Read the status of the IO ports (0x34).
void readIOstatus(uint32_t id);

/// @brief Read the raw 48-bit encoder count (0x35).
void readRawEncoder(uint32_t id);

/// @brief Read the current angle error in ticks (0x39).
void readAngleError(uint32_t id);

/// @brief Read the enable-pin status (0x3A).
void readEnablePin(uint32_t id);

/// @brief Read status of zero-return on power-up (0x3B).
void readZeroStatus(uint32_t id);

/// @brief Release locked-rotor protection state (0x3D).
void releaseProtection(uint32_t id);

/// @brief Read whether protection is active (0x3E).
void readProtectState(uint32_t id);

/// @brief Calibrate the encoder (0x80).
void calibrate(uint32_t id);

/// @brief Set work mode (0=CR_OPEN,…,5=SR_vFOC) (0x82).
void setWorkMode(uint32_t id, uint8_t mode);

/// @brief Set motor current in mA (0x83).
void setCurrent(uint32_t id, uint16_t ma);

/// @brief Set microstepping subdivision (0x84).
void setMicrostep(uint32_t id, uint8_t microsteps);

/// @brief Configure EN pin active level (0=Low,1=High,2=Hold) (0x85).
void setEnActive(uint32_t id, uint8_t level);

/// @brief Set motor direction (0=CW,1=CCW) (0x86).
void setDirection(uint32_t id, uint8_t dir);

/// @brief Enable or disable automatic OLED sleep (0x87).
void setAutoSleep(uint32_t id, bool enable);

/// @brief Enable or disable locked-rotor protection (0x88).
void setProtect(uint32_t id, bool enable);

/// @brief Enable or disable subdivision interpolation (0x89).
void setInterpolator(uint32_t id, bool enable);

/// @brief Set holding current percentage (10%–90%) (0x9B).
void setHoldCurrent(uint32_t id, uint8_t percent);

/// @brief Set CAN bus bit rate (0=125k,1=250k,2=500k,3=1M) (0x8A).
void setCanRate(uint32_t id, uint8_t rate);

/// @brief Change the device’s CAN ID (0x8B).
void setCanId(uint32_t id, uint16_t newId);

/// @brief Enable or disable slave response and active flags (0x8C).
void setCanResponse(uint32_t id, bool respond, bool active);

/// @brief Set group ID for multi-drop commands (0x8D).
void setGroupId(uint32_t id, uint16_t groupId);

/// @brief Lock or unlock the front-panel keys (0x8F).
void setKeylock(uint32_t id, bool lock);

/// @brief Configure homing parameters (trig level, direction, speed, end-limit, mode) (0x90).
void setHomeParams(uint32_t id,
                   uint8_t  triggerLevel,
                   uint8_t  homeDir,
                   uint16_t homeSpeed,
                   bool     endLimit,
                   uint8_t  mode);

/// @brief Execute homing routine (0x91).
void goHome(uint32_t id);

/// @brief Set the current position as zero without moving (0x92).
void setZeroPoint(uint32_t id);

/// @brief Configure “no-limit” return parameters (return angle, current) (0x94).
void setNoLimitReturn(uint32_t id, uint32_t returnAngle, uint16_t ma);

/// @brief Remap limit switches to EN/DIR pins in serial mode (0x9E).
void setLimitRemap(uint32_t id, bool enable);

/// @brief Configure zero-on-power parameters (mode, enable, speed, direction) (0x9A).
void setZeroMode(uint32_t id,
                 uint8_t mode,
                 bool    enable,
                 uint8_t speed,
                 uint8_t dir);

/// @brief Restore default parameters (0x3F).
void restoreDefaults(uint32_t id);

/// @brief Restart the driver firmware (0x41).
void restart(uint32_t id);

/// @brief Configure EN-triggered zeroing and position-error protection (0x9D).
void setEnTrigger(uint32_t id,
                  bool     enTrigger,
                  bool     posProtect,
                  uint16_t timeout,
                  uint16_t errorThreshold);

/// @brief Read an arbitrary system parameter by code (0x00 + code).
void readSystemParam(uint32_t id, uint8_t code);

/// @brief Query motor status (stopped, homing, etc.) (0xF1).
void queryStatus(uint32_t id);

/// @brief Enable or disable the motor outputs (0=off,1=on) (0xF3).
void enableMotor(uint32_t id, bool enable);

/// @brief Perform an emergency stop (0xF7).
void emergencyStop(uint32_t id);

/// @brief Run in speed-mode with given speed, accel, and direction (0xF6).
void speedMode(uint32_t id, uint16_t speed, uint8_t accel, bool ccw);

/// @brief Stop speed-mode (acc=0, speed=0 on 0xF6).
void speedModeStop(uint32_t id);

/// @brief Save or clear speed-mode parameters (0xFF, state 0xC8=Save,0xCA=Clear).
void speedState(uint32_t id, bool save);

/// @brief Move by a relative number of pulses (0xFD).
void posRelative(uint32_t id,
                 uint32_t pulses,
                 uint16_t speed,
                 uint8_t  accel,
                 bool     ccw);

/// @brief Stop a relative-pulse move immediately or with deceleration (0xFD).
void posRelativeStop(uint32_t id);

/// @brief Move to an absolute axis position (0xFE).
void posAbsolute(uint32_t id,
                 int32_t  axis,
                 uint16_t speed,
                 uint8_t  accel);

/// @brief Stop an absolute-position move (0xFE).
void posAbsoluteStop(uint32_t id);

/// @brief Move by a relative encoder-axis offset (0xF4).
void posAxis(uint32_t id,
             int32_t  relAxis,
             uint16_t speed,
             uint8_t  accel);

/// @brief Stop a relative-axis move (0xF4).
void posAxisStop(uint32_t id);

} // namespace MKSServoCAN

