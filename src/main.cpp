#include <iostream>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../icm20948-official/icm20948.h"
#include "../icm20948-official/ICM_20948_C.h"

#define I2C_ADDR ICM_20948_I2C_ADDR_AD1

// Define I2C pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);

    int result = i2c_write_blocking(i2c0, I2C_ADDR, buffer, len + 1, false);
    return (result == PICO_ERROR_GENERIC) ? ICM_20948_Stat_Err : ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    int result = i2c_write_blocking(i2c0, I2C_ADDR, &reg, 1, true);
    if (result == PICO_ERROR_GENERIC) {
        return ICM_20948_Stat_Err;
    }

    result = i2c_read_blocking(i2c0, I2C_ADDR, buff, len, false);
    return (result == PICO_ERROR_GENERIC) ? ICM_20948_Stat_Err : ICM_20948_Stat_Ok;
}

const ICM_20948_Serif_t mySerif = {
    my_write_i2c, // write
    my_read_i2c,  // read
    NULL,
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

    ICM_20948_init_struct(&myICM);
    ICM_20948_link_serif(&myICM, &mySerif);

    while (ICM_20948_check_id(&myICM) != ICM_20948_Stat_Ok) {
      std::cout << "whoami does not match. halting..." << std::endl;
      sleep_ms(1000);
    }

    ICM_20948_Status_e stat = ICM_20948_Stat_Err;
    uint8_t whoami = 0x00;

    while((stat != ICM_20948_Stat_Ok) || (whoami != ICM_20948_WHOAMI)) {
      whoami = 0x00;
      stat = ICM_20948_get_who_am_i(&myICM, &whoami);
      std::cout << "whoami does not match (0x" << whoami << "). halting..." << std::endl;
      sleep_ms(1000);
    }

    ICM_20948_sw_reset(&myICM);
    sleep_ms(250);
    // Set Gyro and Accelerometer to a particular sample mode
    ICM_20948_set_sample_mode(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myfss;
    myfss.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    myfss.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    ICM_20948_set_full_scale(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myfss);

    // Set up DLPF configuration
    ICM_20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = acc_d473bw_n499bw;
    myDLPcfg.g = gyr_d361bw4_n376bw5;
    ICM_20948_set_dlpf_cfg(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

    // Choose whether or not to use DLPF
    ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Acc, false);
    ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Gyr, false);

    // Now wake the sensor up
    ICM_20948_sleep(&myICM, false);
    ICM_20948_low_power(&myICM, false);

    while (1) {
        ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
        if (ICM_20948_get_agmt(&myICM, &agmt) == ICM_20948_Stat_Ok)
        {
          printRawAGMT(agmt);
        }
        else
        {
          std::cout << "Uh oh" << std::endl;
        }

        sleep_ms(1000);
    }

    return 0;
}
