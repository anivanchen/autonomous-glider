#pragma once
#ifndef __PID_H__
#define __PID_H__

class PID {
  public:
    PID(float kp, float ki, float kd);
    float update(float setpoint, float input);
    void setGains(float kp, float ki, float kd);
    void updateDT();
    void reset();
    float getOutput();
  private:
    float kp, ki, kd, dt;
    float lastTime;
    float integral, prevError;
    float output;
};

#endif