#pragma once
#ifndef __MAIN_H__
#define __MAIN_H__

#include <cstring>
#include <iostream>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "icm20948.h"
#include "pid.h"
#include "pwmservo.h"
#include "bmp280.h"

#define LED_PIN 25 // TODO: FILL IN WITH CORRECT PIN

#endif