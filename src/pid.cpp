#include "pid.h"

// PID controller
PID::PID(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->dt = 0;
  this->integral = 0;
  this->prevError = 0;
  this->output = 0;
  this->lastTime = time_us_64() / 1000;
}

// Update PID controller
float PID::update(float setpoint, float measurement) {
  updateDT();
  float error = setpoint - measurement;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  output = kp * error + ki * integral + kd * derivative;
  prevError = error;
  return output;
}

// Set PID gains
void PID::setGains(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

// Set PID time step
void PID::updateDT() {
  uint64_t currentTime = time_us_64() / 1000;
  this->dt = currentTime - lastTime;
  lastTime = currentTime;
}

// Reset PID controller
void PID::reset() {
  integral = 0;
  prevError = 0;
  output = 0;
}

// Get PID output
float PID::getOutput() {
  return output;
}
