#include "main.h"
#include "icm20948.h"

int main() {

  // Initialize chosen serial port
  stdio_init_all();

  // Initialize I2C port at 100 kHz
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  icm20948_init();


  // Primary loop

  while (1) {

    sleep_ms(250);

    icm20948_gyro_t gyroData;

    icm20948_getGyroData(&gyroData);
    icm20948_getTempData();

  }

  return 0;
}
