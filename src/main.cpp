#include <iostream>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../icm20948-official/ICM_20948.h"

#define I2C_ADDR ICM_20948_I2C_ADDR_AD1

// Define I2C pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    i2c_inst_t *i2c = (i2c_inst_t *)user;
    i2c_write_blocking(i2c, I2C_ADDR, &reg, 1, true);
    i2c_write_blocking(i2c, I2C_ADDR, data, len, false);
    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    i2c_inst_t *i2c = (i2c_inst_t *)user;
    i2c_write_blocking(i2c, I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, I2C_ADDR, buff, len, false);
    return ICM_20948_Stat_Ok;
}

const ICM_20948_Serif_t mySerif = {
    ICM_20948_write_I2C, // write
    ICM_20948_read_I2C,  // read
    NULL
};

ICM_20948_Device_t myICM;

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  std::cout << "RAW. Acc [ ";
  std::cout << agmt.acc.axes.x;
  std::cout << ", ";
  std::cout << agmt.acc.axes.y;
  std::cout << ", ";
  std::cout << agmt.acc.axes.z;
  std::cout << " ], Gyr [ ";
  std::cout << agmt.gyr.axes.x;
  std::cout << ", ";
  std::cout << agmt.gyr.axes.y;
  std::cout << ", ";
  std::cout << agmt.gyr.axes.z;
  std::cout << " ], Mag [ ";
  std::cout << agmt.mag.axes.x;
  std::cout << ", ";
  std::cout << agmt.mag.axes.y;
  std::cout << ", ";
  std::cout << agmt.mag.axes.z;
  std::cout << " ], Tmp [ ";
  std::cout << agmt.tmp.val;
  std::cout << " ]" << std::endl;
}

float getAccMG(int16_t raw, uint8_t fss)
{
  switch (fss)
  {
  case 0:
    return (((float)raw) / 16.384);
    break;
  case 1:
    return (((float)raw) / 8.192);
    break;
  case 2:
    return (((float)raw) / 4.096);
    break;
  case 3:
    return (((float)raw) / 2.048);
    break;
  default:
    return 0;
    break;
  }
}

float getGyrDPS(int16_t raw, uint8_t fss)
{
  switch (fss)
  {
  case 0:
    return (((float)raw) / 131);
    break;
  case 1:
    return (((float)raw) / 65.5);
    break;
  case 2:
    return (((float)raw) / 32.8);
    break;
  case 3:
    return (((float)raw) / 16.4);
    break;
  default:
    return 0;
    break;
  }
}

float getMagUT(int16_t raw)
{
  return (((float)raw) * 0.15);
}

float getTmpC(int16_t raw)
{
  return (((float)raw) / 333.87);
}

int main() {
    stdio_init_all();

    ICM_20948_I2C myICM;

    myICM.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    while (1) {
      std::cout << "Hello World" << std::endl;

      sleep_ms(1000);

      // ICM_20948_AGMT_t agmt = myICM.getAGMT();
      // printRawAGMT(agmt);
    }

    // myICM._serif = &mySerif;
    // myICM._user = (void *)&i2c0;

    // ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    // status = ICM_20948_begin(&myICM, &mySerif, ICM_20948_ARD_UNUSED_PIN);

    // if (status != ICM_20948_Stat_Ok)
    // {
    //     std::cout << "ICM_20948_begin returned: " << std::endl;
    //     debugPrintStatus(status);
    //     std::cout << "" << std::endl;
    // }

    // ICM_20948_AGMT_t agmt = ICM_20948_getAGMT(&myICM);
    // printRawAGMT(agmt);
}
