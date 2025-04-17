/**
 * Serial_Test.ino
 *
 * A simple CAN-testing interface for a single MKS SERVO42D/57D motor.
 * Made by Will Hickmott (@TheSpaceEgg) on GitHub.
 */

#include <Arduino.h>
#include <TwaiCan.h>
#include <MKSServoCAN.h>

#define MOTOR_ID 1

TwaiCan bus(GPIO_NUM_27, GPIO_NUM_26);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2500);

  if (!MKSServoCAN::begin(&bus)) {
    Serial.println("CAN init failed");
    while (1);
  }
  Serial.println("CAN up @500 kbps\n");

  Serial.println(
    "Commands:\n"
    " h                → Go home\n"
    " e                → Enable motor\n"
    " x                → Disable motor\n"
    " m <p> <s> <a> <d> → Move relative (p=pulses, s=speed, a=accel, d=0=CW,1=CCW)\n"
    " a <axis> <s> <a> <d> → Move absolute (axis steps, s=speed, a=accel, d=0=CW,1=CCW)\n"
    " r <hex>          → Read system param (e.g. r 82)\n"
    " i                → Read encoder carry\n"
    " p                → Read pulses received\n"
    " s                → Read speed\n"
    " o <0-3>          → Set CAN bitrate (0=125k,1=250k,2=500k,3=1M)\n"
    " u <r> <a>        → Set CAN respond & active flags\n"
    " ?                → This help\n"
  );
}

void loop() {
  if (!Serial.available()) {
    MKSServoCAN::pollResponses();
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    MKSServoCAN::pollResponses();
    return;
  }

  char buf[64];
  line.toCharArray(buf, sizeof(buf));
  char* tok = strtok(buf, " ");
  if (!tok) {
    MKSServoCAN::pollResponses();
    return;
  }

  if (strcasecmp(tok, "?") == 0 || strcasecmp(tok, "help") == 0) {
    Serial.println(
      "\nCommands:\n"
      " h                → Go home\n"
      " e                → Enable motor\n"
      " x                → Disable motor\n"
      " m <p> <s> <a> <d> → Move relative (p=pulses, s=speed, a=accel, d=0=CW,1=CCW)\n"
      " a <axis> <s> <a> <d> → Move absolute (axis steps, s=speed, a=accel, d=0=CW,1=CCW)\n"
      " r <hex>          → Read system param (e.g. r 82)\n"
      " i                → Read encoder carry\n"
      " p                → Read pulses received\n"
      " s                → Read speed\n"
      " o <0-3>          → Set CAN bitrate (0=125k,1=250k,2=500k,3=1M)\n"
      " u <r> <a>        → Set CAN respond & active flags\n"
      " ?                → This help\n"
    );
  }
  else if (strcasecmp(tok, "h") == 0) {
    MKSServoCAN::goHome(MOTOR_ID);
  }
  else if (strcasecmp(tok, "e") == 0) {
    MKSServoCAN::enableMotor(MOTOR_ID, true);
  }
  else if (strcasecmp(tok, "x") == 0) {
    MKSServoCAN::enableMotor(MOTOR_ID, false);
  }
  else if (strcasecmp(tok, "m") == 0) {
    char* p_tok = strtok(nullptr, " ");
    char* s_tok = strtok(nullptr, " ");
    char* a_tok = strtok(nullptr, " ");
    char* d_tok = strtok(nullptr, " ");
    if (p_tok && s_tok && a_tok && d_tok) {
      uint32_t pulses   = strtoul(p_tok, nullptr, 10);
      uint16_t speed    = uint16_t(strtoul(s_tok, nullptr, 10));
      uint8_t  accel    = uint8_t(strtoul(a_tok, nullptr, 10));
      bool     ccw      = (strtol(d_tok, nullptr, 10) != 0);
      MKSServoCAN::posRelative(MOTOR_ID, pulses, speed, accel, ccw);
    } else {
      Serial.println("Usage: m <pulses> <speed> <accel> <dir 0/1>");
    }
  }
  else if (strcasecmp(tok, "a") == 0) {
    char* ax_tok = strtok(nullptr, " ");
    char* s_tok  = strtok(nullptr, " ");
    char* a_tok  = strtok(nullptr, " ");
    char* d_tok  = strtok(nullptr, " ");
    if (ax_tok && s_tok && a_tok && d_tok) {
      int32_t axis     = strtol(ax_tok, nullptr, 10);
      uint16_t speed   = uint16_t(strtoul(s_tok, nullptr, 10));
      uint8_t  accel   = uint8_t(strtoul(a_tok, nullptr, 10));
      bool     ccw     = (strtol(d_tok, nullptr, 10) != 0);
      if (ccw) axis = -axis;
      MKSServoCAN::posAbsolute(MOTOR_ID, axis, speed, accel);
    } else {
      Serial.println("Usage: a <axis> <speed> <accel> <dir 0/1>");
    }
  }
  else if (strcasecmp(tok, "i") == 0) {
    MKSServoCAN::readEncoderCarry(MOTOR_ID);
  }
  else if (strcasecmp(tok, "p") == 0) {
    MKSServoCAN::readPulses(MOTOR_ID);
  }
  else if (strcasecmp(tok, "s") == 0) {
    MKSServoCAN::readSpeed(MOTOR_ID);
  }
  else if (strcasecmp(tok, "r") == 0) {
    char* c_tok = strtok(nullptr, " ");
    if (c_tok) {
      uint8_t code = (uint8_t)strtol(c_tok, nullptr, 16);
      MKSServoCAN::readSystemParam(MOTOR_ID, code);
    } else {
      Serial.println("Usage: r <hex-param-code>");
    }
  }
  else if (strcasecmp(tok, "o") == 0) {
    char* r_tok = strtok(nullptr, " ");
    if (r_tok) {
      uint8_t rate = uint8_t(strtoul(r_tok, nullptr, 10));
      MKSServoCAN::setCanRate(MOTOR_ID, rate);
    } else {
      Serial.println("Usage: o <0-3>");
    }
  }
  else if (strcasecmp(tok, "u") == 0) {
    char* resp_tok = strtok(nullptr, " ");
    char* act_tok  = strtok(nullptr, " ");
    if (resp_tok && act_tok) {
      bool resp = (bool)strtol(resp_tok, nullptr, 10);
      bool act  = (bool)strtol(act_tok,  nullptr, 10);
      MKSServoCAN::setCanResponse(MOTOR_ID, resp, act);
    } else {
      Serial.println("Usage: u <resp(0/1)> <act(0/1)>");
    }
  }
  else {
    Serial.println("Unknown command - type '?' for help");
  }

  MKSServoCAN::pollResponses();
}
