#include "pwmservo.h"

Servo::Servo(uint8_t servoPin) {
  this->servoPin = servoPin;
  this->currentPosition = 0;

  gpio_set_function(servoPin, GPIO_FUNC_PWM);

  uint slice_num = pwm_gpio_to_slice_num(servoPin);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 16.0f);
  pwm_set_wrap(slice_num, (125000000 / (16.0f * 50.0f)) - 1);
  pwm_init(slice_num, &config, true);
}

// Minimum: 900, Maximum: 2100, Midpoint: 1500

void Servo::setPosition(uint16_t position) {
  uint16_t level = (position * (125000000 / (16.0f * 50.0f)) - 1) /
                   (1000000 / (125000000 / 16.0f / ((125000000 / (16.0f * 50.0f)) - 1) + 1));
  pwm_set_gpio_level(servoPin, level);
  currentPosition = position;
}

uint16_t Servo::getPosition() { return this->currentPosition; }