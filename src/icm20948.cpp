#include "icm20948.h"

icm20948_return_code_t icm20948_init() {

  sleep_ms(500); // Short delay for boot up

  // Initialize I2C port at 400 kHz
  i2c_init(IMU_I2C_PORT, 400 * 1000);

  // Set pin functions
  gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(IMU_I2C_SDA_PIN);
  gpio_pull_up(IMU_I2C_SCL_PIN);

  uint8_t reg = B0_WHO_AM_I;
  uint8_t chipID[1];
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, chipID, 1, false);

  if (chipID[0] != ICM20948_WHO_AM_I_DEFAULT) {
    printf("Chip ID DOES NOT MATCH 0xEA: 0x%02X\n", chipID[0]);
    return ICM20948_RET_GEN_FAIL;
  }

  printf("ICM20948 detected\n");
  printf("Chip ID MATCHES 0xEA: 0x%02X\n", chipID[0]);

  // Set clock source
  uint8_t data[2] = {B0_PWR_MGMT_1, 0x01};
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Ensure accelerometer and gyroscope are enabled
  data[0] = B0_PWR_MGMT_2;
  data[1] = 0x00;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Change to user bank 2
  data[0] = ICM20948_REG_BANK_SEL;
  data[1] = ICM20948_USER_BANK_2;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Configure gyro
  data[0] = B2_GYRO_CONFIG_1;
  data[1] = 0x1D; // 1000dps, 73.3 NBW (hz), active low pass filter
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Set gyro rate
  data[0] = B2_GYRO_SMPLRT_DIV;
  data[1] = 0x00; // 1.1kHz / (1 + 0) = 1.1kHz
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Configure accel
  data[0] = B2_ACCEL_CONFIG;
  data[1] = 0x1D; // 8g, 68.8 NBW (hz), active low pass filter
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Set accel rate
  data[0] = B2_ACCEL_SMPLRT_DIV_1;
  data[1] = 0x00;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  data[0] = B2_ACCEL_SMPLRT_DIV_2;
  data[1] = 0x00; // 1.1kHz / (1 + 0) = 1.1kHz
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Configure temp
  data[0] = B2_TEMP_CONFIG;
  data[1] = 0x02;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Change to user bank 0
  data[0] = ICM20948_REG_BANK_SEL;
  data[1] = ICM20948_USER_BANK_0;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Enable Master I2C
  data[0] = B0_USER_CTRL;
  data[1] = 0x20;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  data[0] = B0_LP_CONFIG;
  data[1] = 0x40;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Change to user bank 3
  data[0] = ICM20948_REG_BANK_SEL;
  data[1] = ICM20948_USER_BANK_3;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Configure I2C Master to read magnetometer data 
  data[0] = B3_I2C_MST_ODR_CONFIG;
  data[1] = 0x03;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  // Change to user bank 0
  data[0] = ICM20948_REG_BANK_SEL;
  data[1] = ICM20948_USER_BANK_0;
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 2, true);

  return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_getGyroData(icm20948_gyro_t *data) {

  uint8_t reg = B0_INT_STATUS_1;
  uint8_t ready[1];

  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, ready, 1, false);

  if (ready[0] == 1) {
    printf("Gyro data ready\n");

    reg = B0_GYRO_XOUT_H;
    uint8_t gyroData[6];
    i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, gyroData, 6, false);

    data->raw_x = (int16_t)(gyroData[0] << 8 | gyroData[1]);
    data->raw_y = (int16_t)(gyroData[2] << 8 | gyroData[3]);
    data->raw_z = (int16_t)(gyroData[4] << 8 | gyroData[5]);

    data->x = (int16_t)(data->raw_x * 1000 / 32768.0);
    data->y = (int16_t)(data->raw_y * 1000 / 32768.0);
    data->z = (int16_t)(data->raw_z * 1000 / 32768.0);

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

  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, ready, 1, false);

  if (ready[0] == 1) {
    printf("Accel data ready\n");

    reg = B0_ACCEL_XOUT_H;
    uint8_t accelData[6];
    i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, accelData, 6, false);

    data->raw_x = (int16_t)(accelData[0] << 8 | accelData[1]);
    data->raw_y = (int16_t)(accelData[2] << 8 | accelData[3]);
    data->raw_z = (int16_t)(accelData[4] << 8 | accelData[5]);

    data->x = (int16_t)(data->raw_x * 8 / 4096.0);
    data->y = (int16_t)(data->raw_y * 8 / 4096.0);
    data->z = (int16_t)(data->raw_z * 8 / 4096.0);

    printf("X: %d, Y: %d, Z: %d\n", data->x, data->y, data->z);
  }

  if (ready[0] == 0) {
    printf("Accel data not ready\n");
  }

  return ICM20948_RET_OK;
}

int ak09916_write_reg(uint8_t reg, uint8_t data) {
  i2c_write_blocking(IMU_I2C_PORT, AK09916_ADDR, &reg, 1, true);
  i2c_write_blocking(IMU_I2C_PORT, AK09916_ADDR, &data, 1, true);
  i2c_write_blocking(IMU_I2C_PORT, AK09916_ADDR, &data, 1, false);
}

int ak09916_read_reg(uint8_t reg, uint8_t *data) {
  i2c_write_blocking(IMU_I2C_PORT, AK09916_ADDR, &reg, 1, true);
  i2c_write_blocking(IMU_I2C_PORT, AK09916_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, AK09916_ADDR, data, 1, false);
}

icm20948_return_code_t icm20948_getData(icm20948_gyro_t *gyro_data, icm20948_accel_t *accel_data) {

  uint8_t reg = B0_INT_STATUS_1;
  uint8_t ready[1];

  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, ready, 1, false);

  if (ready[0] == 1) {

    // printf("Data Ready\n");

    reg = B0_ACCEL_XOUT_H;
    uint8_t raw_data[12];
    i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, raw_data, 12, false);

    gyro_data->raw_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    gyro_data->raw_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    gyro_data->raw_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    gyro_data->x = (int16_t)(gyro_data->raw_x * 1000 / 32768.0);
    gyro_data->y = (int16_t)(gyro_data->raw_y * 1000 / 32768.0);
    gyro_data->z = (int16_t)(gyro_data->raw_z * 1000 / 32768.0);

    printf("X: %d, Y: %d, Z: %d\n", gyro_data->x, gyro_data->y, gyro_data->z);

    accel_data->raw_x = (int16_t)(raw_data[6] << 8 | raw_data[7]);
    accel_data->raw_y = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    accel_data->raw_z = (int16_t)(raw_data[10] << 8 | raw_data[11]);

    accel_data->x = (int16_t)(accel_data->raw_x * 8 / 4096.0);
    accel_data->y = (int16_t)(accel_data->raw_y * 8 / 4096.0);
    accel_data->z = (int16_t)(accel_data->raw_z * 8 / 4096.0);

    printf("X: %d, Y: %d, Z: %d\n", accel_data->x, accel_data->y, accel_data->z);
    // printf("%d,%d,%d\n", accel_data->x, accel_data->y, accel_data->z);

  }

  // if (ready[0] == 0) {
  //   printf("Data not ready\n");
  // }

  return ICM20948_RET_OK;

}

icm20948_return_code_t icm20948_getMagData(icm20948_mag_t *data) {

  return ICM20948_RET_OK;

}

icm20948_return_code_t icm20948_getTempData() {

  uint8_t reg = B0_TEMP_OUT_H;
  uint8_t tempData[2];
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, tempData, 2, false);

  uint16_t raw_temp = (int16_t)(tempData[0] << 8 | tempData[1]);
  int16_t temp_c = (int16_t)(raw_temp / 333.87 + 21.0);

  printf("Temp: %d\n", temp_c);

  return ICM20948_RET_OK;
}

icm20948_return_code_t enable_dmp() {
  uint8_t reg = B0_USER_CTRL;
  uint8_t data[1] = {0xC0};
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 1, false);
  return ICM20948_RET_OK;
}

icm20948_return_code_t disable_dmp() {
  uint8_t reg = B0_USER_CTRL;
  uint8_t data[1] = {0x00};
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 1, false);
  return ICM20948_RET_OK;
}

icm20948_return_code_t read_dmp() {
  uint8_t reg = B0_EXT_SLV_SENS_DATA_00;
  uint8_t data[1];
  i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, &reg, 1, true);
  i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDR, data, 1, false);
  printf("DMP Data: %d\n", data[0]);
  return ICM20948_RET_OK;
}
