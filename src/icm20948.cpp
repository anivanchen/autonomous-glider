#include "icm20948.h"

icm20948_return_code_t icm20948_init() {
  sleep_ms(1000); // Short delay for boot up
  uint8_t reg = B0_WHO_AM_I;
  uint8_t chipID[1];
  i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(I2C_PORT, I2C_ADDR, chipID, 1, false);

  if (chipID[0] != ICM20948_WHO_AM_I_DEFAULT) {
    printf("Chip ID DOES NOT MATCH 0xEA: 0x%02X\n", chipID[0]);
    return ICM20948_RET_GEN_FAIL;
  }

  printf("ICM20948 detected\n");
  printf("Chip ID MATCHES 0xEA: 0x%02X\n", chipID[0]);

  // Set clock source

  uint8_t data[2] = {B0_PWR_MGMT_1, 0x01};
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  // Ensure accelerometer and gyroscope are enabled

  data[0] = B0_PWR_MGMT_2;
  data[1] = 0x00;
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  // Change to user bank 2
  data[0] = ICM20948_REG_BANK_SEL;
  data[1] = ICM20948_USER_BANK_2;
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  // Configure gyro
  data[0] = B2_GYRO_CONFIG_1;
  data[1] = 0x29; // 250dps
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  // Set gyro rate
  data[0] = B2_GYRO_SMPLRT_DIV;
  data[1] = 0x00;
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  // Configure accel
  data[0] = B2_ACCEL_CONFIG;
  data[1] = 0x29;

  // Set accel rate
  data[0] = B2_ACCEL_SMPLRT_DIV_1;
  data[1] = 0x00;
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  data[0] = B2_ACCEL_SMPLRT_DIV_2;
  data[1] = 0x00;
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  // Change to user bank 0
  data[0] = ICM20948_REG_BANK_SEL;
  data[1] = ICM20948_USER_BANK_0;
  i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 2, true);

  return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_getGyroData(icm20948_gyro_t *data) {

  uint8_t reg = B0_INT_STATUS_1;
  uint8_t ready[1];

  i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(I2C_PORT, I2C_ADDR, ready, 1, false);

  if (ready[0] == 1) {
    printf("Gyro data ready\n");

    reg = B0_GYRO_XOUT_H;
    uint8_t gyroData[6];
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, I2C_ADDR, gyroData, 6, false);

    data->raw_x = (int16_t)(gyroData[0] << 8 | gyroData[1]);
    data->raw_y = (int16_t)(gyroData[2] << 8 | gyroData[3]);
    data->raw_z = (int16_t)(gyroData[4] << 8 | gyroData[5]);

    data->x = (int16_t)(data->raw_x * 250 / 32768.0);
    data->y = (int16_t)(data->raw_y * 250 / 32768.0);
    data->z = (int16_t)(data->raw_z * 250 / 32768.0);

    printf("X: %d, Y: %d, Z: %d\n", data->x, data->y, data->z);

  }

  if (ready[0] == 0) {
    printf("Gyro data not ready\n");
  }
  
  return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_getAccelData(icm20948_accel_t *data) {

  uint8_t reg = B0_INT_STATUS_1;
  uint8_t ready[1];

  i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(I2C_PORT, I2C_ADDR, ready, 1, false);

  if (ready[0] == 1) {
    printf("Accel data ready\n");

    reg = B0_ACCEL_XOUT_H;
    uint8_t accelData[6];
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, I2C_ADDR, accelData, 6, false);

    data->raw_x = (int16_t)(accelData[0] << 8 | accelData[1]);
    data->raw_y = (int16_t)(accelData[2] << 8 | accelData[3]);
    data->raw_z = (int16_t)(accelData[4] << 8 | accelData[5]);

    data->x = (int16_t)(data->raw_x * 250 / 32768.0);
    data->y = (int16_t)(data->raw_y * 250 / 32768.0);
    data->z = (int16_t)(data->raw_z * 250 / 32768.0);

    printf("X: %d, Y: %d, Z: %d\n", data->x, data->y, data->z);
  }

  if (ready[0] == 0) {
    printf("Accel data not ready\n");
  }

  return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_getMagData(icm20948_mag_t *data) {

}

icm20948_return_code_t icm20948_getTempData() {

  uint8_t reg = B0_TEMP_OUT_H;
  uint8_t tempData[2];
  i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(I2C_PORT, I2C_ADDR, tempData, 6, false);

  printf("Temp: %d\n", tempData[1]);

  return ICM20948_RET_OK;
}
