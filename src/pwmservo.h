#pragma once
#ifndef __PWM_SERVO_H__
#define __PWM_SERVO_H__

#include <stdint.h>

#include <cstddef>
#include <iostream>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define SERVO_FREQ  50
#define SERVO_RANGE 39062.0f

class Servo {
 public:
  Servo(uint8_t servoPin);
  void setPosition(uint16_t position);
  uint16_t getPosition();

 private:
  uint servoPin;
  uint16_t currentPosition;
};

#endif