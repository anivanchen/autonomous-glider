#include "main.h"

enum glider_state {
  STEADY_FLIGHT,
  BANKING_180,
  CONTROLLED_DESCENT,
  LANDING
};

int main() {
  
  /*** Serial Port Initialization ***/
  stdio_init_all();

  /*** State Machine Initialization ***/

  glider_state current_state = STEADY_FLIGHT;

  // GPS initialization



  /*** IMU Initialization ***/
  
  icm20948_init();

  /*** Barometer Initialization ***/

  bmp280_init();

  /*** Servo Initialization ***/

  Servo leftAileronServo = Servo(29);
  Servo rightAileronServo = Servo(28);
  Servo elevatorServo = Servo(27);

  /*** PID Initialization ***/

  PID pitchPID = PID(0.1, 0.1, 0.1);
  PID rollPID = PID(0.1, 0.1, 0.1);

  // Primary loop

  while (1) {

    switch (current_state) {
      case STEADY_FLIGHT:
        // get pitch to be around 0 +- 5 degrees
        // get roll to be around 0 +- 5 degrees

        // if steady flight for 5 seconds, switch to banking 180
        // current_state = BANKING_180;

        break;
      case BANKING_180:
        // allow pitch to descend, 5 degreesish
        // start rolling right until 180 degrees
        // maximum roll rate is 3 degrees per second

        // if 180 degrees, switch to controlled descent
        // current_state = CONTROLLED_DESCENT;

        break;
      case CONTROLLED_DESCENT:
        // pitch down 10 degrees to start descent
        // roll left/right to be on course with the goal

        // at a certain altitude, switch to landing
        // current_state = LANDING;

        break;

      case LANDING:
        // attempt to slow down by pitching up 30 degrees
        break;
    }

  }

  return 0;
}
