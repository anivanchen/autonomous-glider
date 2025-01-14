#include "main.h"
#include "icm20948.h"
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

  // Initialize PID Controller

  PID pid(1, 0, 0, 0.02)

  // Primary loop

  while (1) {

    // icm20948_gyro_t gyro_data;
    // icm20948_accel_t accel_data;

    // icm20948_getData(&gyro_data, &accel_data);
    // icm20948_getTempData();

  }

  return 0;
}
