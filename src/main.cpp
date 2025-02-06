#include "main.h"

/*
 * UNIT CONVENTIONS
 * DISTANCE = METERS
 * ANGLE = DEGREES
 * TIME = SECONDS
 */

enum glider_state {
  STEADY_FLIGHT,
  BANKING_180,
  CONTROLLED_DESCENT,
  LANDING
};

struct pose {
  float x;
  float y;
  float z;
  float pitch;
  float roll;
  float yaw;
};

struct pose getPose() {
  struct pose current_pose;
  current_pose.x = 0;
  current_pose.y = 0;
  current_pose.z = 0;
  current_pose.pitch = 0;
  current_pose.roll = 0;
  current_pose.yaw = 0;
  return current_pose;
}

bool withinTolerance(float value, float target, float tolerance) {
  return value > target - tolerance && value < target + tolerance;
}

void blink_lights() {
  if (gpio_get(LED_PIN)) {
    gpio_put(LED_PIN, 0);
  } else {
    gpio_put(LED_PIN, 1);
  }
}

int main() {
  
  /*** Serial Port Initialization ***/
  stdio_init_all();

  /*** State Machine Initialization ***/

  glider_state current_state = STEADY_FLIGHT;
  glider_state previous_state = current_state;

  // GPS initialization

  /*** IMU Initialization ***/
  
  icm20948_init();

  /*** Barometer Initialization ***/

  // bmp280_init();

  /*** Servo Initialization ***/

  Servo leftAileronServo = Servo(29);
  Servo rightAileronServo = Servo(28);
  Servo elevatorServo = Servo(27);

  /*** PID Initialization ***/

  PID pitchPID = PID(0.1, 0.1, 0.1);
  PID rollPID = PID(0.1, 0.1, 0.1);

  // Time tracker for LEDS

  float led_blinker_time = time_us_64() / 1000000;
  float state_transition_time = time_us_64() / 1000000;

  // Transition Flags

  bool isSameState = false;
  bool steady_flight_transition = false;

  // Primary loop

  while (1) {

    pose current_pose = getPose();
    float current_time = time_us_64() / 1000000;

    isSameState = current_state == previous_state; 
    previous_state = current_state;

    switch (current_state) {
      case STEADY_FLIGHT:
      {
        if (!isSameState) {
          state_transition_time = current_time;
          steady_flight_transition = false;
        }

        // get pitch to be around 0 +- 5 degrees
        // get roll to be around 0 +- 5 degrees

        if (!withinTolerance(current_pose.pitch, 0, 5)) {
          pitchPID.update(0, current_pose.pitch);
        } 

        if (!withinTolerance(current_pose.roll, 0, 5)) {
          rollPID.update(0, current_pose.roll);
        }

        // if steady flight for 5 seconds, switch to banking 180

        steady_flight_transition = withinTolerance(current_pose.pitch, 0, 5);

        if (steady_flight_transition && state_transition_time - current_time >= 5) {
          current_state = BANKING_180;
          steady_flight_transition = false;
        }

        break;
      }

      case BANKING_180:
      {
        if (!isSameState) {
          state_transition_time = current_time;
        }
        // allow pitch to descend, 5 degreesish
        // start rolling right until yaw is 180 degrees
        // maximum roll rate is 3 degrees per second

        if (!withinTolerance(current_pose.pitch, 5, 5)) {
          pitchPID.update(5, current_pose.pitch);
        } else {
          pitchPID.update(0, current_pose.pitch);
        }

        if (!withinTolerance(current_pose.yaw, 180, 5)) {
          rollPID.update(5, current_pose.roll);
        } else {
          rollPID.update(0, current_pose.roll);
        }

        // if has rotated 180 degrees +- 10 degrees and within 40 meters 
        // of target y, switch to controlled descent
        if (withinTolerance(current_pose.y, 0, 20) && withinTolerance(current_pose.yaw, 180, 10)) {
          current_state = CONTROLLED_DESCENT;
        }

        break;
      }

      case CONTROLLED_DESCENT:
      {
        if (!isSameState) {
          state_transition_time = current_time;
        }

        // pitch down 10 degrees to start descent
        // roll left/right to be on course with the goal
        if (withinTolerance(current_pose.y, 0, 20)) {
          pitchPID.update(10, current_pose.pitch);
        } else {
          pitchPID.update(0, current_pose.pitch);
        }

        if (!withinTolerance(current_pose.roll, 0, 5)) {
          rollPID.update(0, current_pose.roll);
        }

        // once below 15 meters altitude, switch to landing
        if (current_pose.z <= 15) {
          current_state = LANDING;
        }

        break;
      }

      case LANDING:
      {
        // attempt to slow down by pitching up 30 degrees

        if (!withinTolerance(current_pose.pitch, 30, 5)) {
          pitchPID.update(30, current_pose.pitch);
        } else {
          pitchPID.update(0, current_pose.pitch);
        }

        break;
      }
    }

    // Update servos

    leftAileronServo.setPosition(rollPID.getOutput());
    rightAileronServo.setPosition(rollPID.getOutput());
    elevatorServo.setPosition(pitchPID.getOutput());

    // Update LEDS

    if (current_time - led_blinker_time > 1) {
      blink_lights();
      led_blinker_time = current_time;
    }

  }

  return 0;
}
