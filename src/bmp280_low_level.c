/*BMP280 barometric pressure and temperature sensor C Driver*/
/*Reza Ebrahimi - https://github.com/ebrezadev */
/*Version 1.0*/

/*Adapted for RP2040 by Ivan Chen - https://github.com/anivanchen*/

#include "bmp280.h"

// Define I2C pins
#define BARO_I2C_PORT i2c1
#define BARO_I2C_ADDR 0x76
#define BARO_I2C_SDA_PIN 2
#define BARO_I2C_SCL_PIN 3

/*writes an array (data[]) of arbitrary size (dataLength) to I2C address (deviceAddress), starting from an internal register address (startRegisterAddress)*/
void bmp280_write_array(uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength)
{
  i2c_write_blocking(BARO_I2C_PORT, deviceAddress, &startRegisterAddress, 1, true);
  i2c_write_blocking(BARO_I2C_PORT, deviceAddress, data, dataLength, false);
}

/*reads an array (data[]) of arbitrary size (dataLength) from I2C address (deviceAddress), starting from an internal register address (startRegisterAddress)*/
void bmp280_read_array (uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength)
{
  i2c_write_blocking(BARO_I2C_PORT, deviceAddress, startRegisterAddress, 1, true);
  i2c_read_blocking(BARO_I2C_PORT, deviceAddress, data, dataLength, false);
}

/*initiates the I2C peripheral and sets its speed*/
void bmp280_i2c_init()        
{
  sleep_ms(500); // Short delay for boot up

  i2c_init(I2C_PORT, 400 * 1000); // TODO: determine what this refresh rate should be for the BMP280
  gpio_set_function(BARO_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(BARO_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(BARO_I2C_SDA_PIN);
  gpio_pull_up(BARO_I2C_SCL_PIN);
}

/*a delay function for milliseconds delay*/
void delay_function (uint32_t delayMS)
{
  sleep_ms(delayMS);
}

/*implements a power function (used in altitude calculation)*/
float power_function (float x, float y)
{
  return pow(x, y);
}