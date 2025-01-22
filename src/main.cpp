#include "main.h"
#include "icm20948.h"
#include "pwmservo.h"
#include "pid.h"

int main() {

  // Initialize chosen serial port
  // stdio_init_all();

  // // Initialize I2C port at 100 kHz
  // i2c_init(I2C_PORT, 400 * 1000);
  // gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  // gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  // gpio_pull_up(I2C_SDA_PIN);
  // gpio_pull_up(I2C_SCL_PIN);

  // icm20948_init();
  // enable_dmp();

  Servo leftAileronServo = Servo(29);
  // Servo rightAileronServo = Servo(3);
  // Servo elevatorServo = Servo(4);

  // Initialize PID Controller

  PID pid(1, 0, 0);

  // Primary loop

  while (1) {

    // icm20948_gyro_t gyro_data;
    // icm20948_accel_t accel_data;

    // icm20948_getData(&gyro_data, &accel_data);
    // read_dmp();

    // leftAileronServo.setPosition(900); // MIN
    // sleep_ms(1000);
    // leftAileronServo.setPosition(2100); // MAX
    // sleep_ms(1000);
    // leftAileronServo.setPosition(1500); // CENTER
    // sleep_ms(1000);

    for (int i = 900; i < 2100; i += 50) {
      leftAileronServo.setPosition(i);
      sleep_ms(50);
      if (i == 2100) {
        i = 900;
      }
    }

    // rightAileronServo.setPosition(50);
    // elevatorServo.setPosition(50);
    
    // icm20948_getTempData();

  }

  return 0;
}
