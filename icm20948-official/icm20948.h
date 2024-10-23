#ifndef ICM_20948_H
#define ICM_20948_H

#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"

#include "ICM_20948_C.h"
#include "AK09916_REGISTERS.h"

/************************/
/* Register Definitions */
/************************/

// clang-format off

#define ICM_20948_ARD_UNUSED_PIN    0xFF

#define ICM20948_USER_BANK_0        0x00
#define ICM20948_USER_BANK_1        0x01
#define ICM20948_USER_BANK_2        0x02
#define ICM20948_USER_BANK_3        0x03

#define ICM20948_REG_BANK_SEL		    0x7F

// User Bank 0

#define B0_WHO_AM_I		              0x00
#define B0_USER_CTRL		            0x03
#define B0_LP_CONFIG		            0x05
#define B0_PWR_MGMT_1		            0x06
#define B0_PWR_MGMT_2		            0x07
#define B0_INT_PIN_CFG	          	0x0F
#define B0_INT_ENABLE		            0x10
#define B0_INT_ENABLE_1	          	0x11
#define B0_INT_ENABLE_2	          	0x12
#define B0_INT_ENABLE_3	          	0x13
#define B0_I2C_MST_STATUS	          0x17
#define B0_INT_STATUS		            0x19
#define B0_INT_STATUS_1		          0x1A
#define B0_INT_STATUS_2		          0x1B
#define B0_INT_STATUS_3		          0x1C
#define B0_DELAY_TIMEH		          0x28
#define B0_DELAY_TIMEL		          0x29

#define B0_ACCEL_XOUT_H		          0x2D
#define B0_ACCEL_XOUT_L		          0x2E
#define B0_ACCEL_YOUT_H		          0x2F
#define B0_ACCEL_YOUT_L		          0x30
#define B0_ACCEL_ZOUT_H		          0x31
#define B0_ACCEL_ZOUT_L		          0x32

#define B0_GYRO_XOUT_H		          0x33
#define B0_GYRO_XOUT_L		          0x34
#define B0_GYRO_YOUT_H		          0x35
#define B0_GYRO_YOUT_L		          0x36
#define B0_GYRO_ZOUT_H		          0x37
#define B0_GYRO_ZOUT_L		          0x38

#define B0_TEMP_OUT_H		            0x39
#define B0_TEMP_OUT_L		            0x3A

#define B0_EXT_SLV_SENS_DATA_00     0x3B
#define B0_EXT_SLV_SENS_DATA_01     0x3C
#define B0_EXT_SLV_SENS_DATA_02     0x3D
#define B0_EXT_SLV_SENS_DATA_03     0x3E
#define B0_EXT_SLV_SENS_DATA_04     0x3F
#define B0_EXT_SLV_SENS_DATA_05     0x40
#define B0_EXT_SLV_SENS_DATA_06     0x41
#define B0_EXT_SLV_SENS_DATA_07     0x42
#define B0_EXT_SLV_SENS_DATA_08     0x43
#define B0_EXT_SLV_SENS_DATA_09     0x44
#define B0_EXT_SLV_SENS_DATA_10     0x45
#define B0_EXT_SLV_SENS_DATA_11     0x46
#define B0_EXT_SLV_SENS_DATA_12     0x47
#define B0_EXT_SLV_SENS_DATA_13     0x48
#define B0_EXT_SLV_SENS_DATA_14     0x49
#define B0_EXT_SLV_SENS_DATA_15     0x4A
#define B0_EXT_SLV_SENS_DATA_16     0x4B
#define B0_EXT_SLV_SENS_DATA_17     0x4C
#define B0_EXT_SLV_SENS_DATA_18     0x4D
#define B0_EXT_SLV_SENS_DATA_19     0x4E
#define B0_EXT_SLV_SENS_DATA_20     0x4F
#define B0_EXT_SLV_SENS_DATA_21     0x50
#define B0_EXT_SLV_SENS_DATA_22     0x51
#define B0_EXT_SLV_SENS_DATA_23     0x52

#define B0_FIFO_EN_1		            0x66
#define B0_FIFO_EN_2		            0x67
#define B0_FIFO_RST		              0x68
#define B0_FIFO_MODE	            	0x69
#define B0_FIFO_COUNTH	           	0x70
#define B0_FIFO_COUNTL		          0x71
#define B0_FIFO_R_W		              0x72
#define B0_DATA_RDY_STATUS	        0x74
#define B0_FIFO_CFG		              0x76

// User Bank 1

#define B1_SELF_TEST_X_GYRO       	0x02
#define B1_SELF_TEST_Y_GYRO	        0x03
#define B1_SELF_TEST_Z_GYRO	        0x04
#define B1_SELF_TEST_X_ACCEL      	0x0E
#define B1_SELF_TEST_Y_ACCEL	      0x0F
#define B1_SELF_TEST_Z_ACCEL	      0x10

#define B1_XA_OFFS_H		            0x14
#define B1_XA_OFFS_L		            0x15
#define B1_YA_OFFS_H		            0x17
#define B1_YA_OFFS_L	            	0x18
#define B1_ZA_OFFS_H	            	0x1A
#define B1_ZA_OFFS_L		            0x1B

#define B1_TIMEBASE_CORRECTION_PLL	0x28

// User Bank 2

#define B2_GYRO_SMPLRT_DIV	        0x00
#define B2_GYRO_CONFIG_1	          0x01
#define B2_GYRO_CONFIG_2	          0x02

#define B2_XG_OFFS_USRH		          0x03
#define B2_XG_OFFS_USRL		          0x04
#define B2_YG_OFFS_USRH		          0x05
#define B2_YG_OFFS_USRL		          0x06
#define B2_ZG_OFFS_USRH		          0x07
#define B2_ZG_OFFS_USRL		          0x08
#define B2_ODR_ALIGN_EN		          0x09

#define B2_ACCEL_SMPLRT_DIV_1	      0x10
#define B2_ACCEL_SMPLRT_DIV_2	      0x11
#define B2_ACCEL_INTEL_CTRL	        0x12
#define B2_ACCEL_WOM_THR	          0x13
#define B2_ACCEL_CONFIG	            0x14
#define B2_ACCEL_CONFIG_2	          0x15

#define B2_FSYNC_CONFIG	            0x52
#define B2_TEMP_CONFIG	            0x53
#define B2_MOD_CTRL_USR	            0x54

// User Bank 3

#define B3_I2C_MST_ODR_CONFIG	      0x00
#define B3_I2C_MST_CTRL	            0x01
#define B3_I2C_MST_DELAY_CTRL	      0x02
#define B3_I2C_SLV0_ADDR	          0x03
#define B3_I2C_SLV0_REG	            0x04
#define B3_I2C_SLV0_CTRL	          0x05
#define B3_I2C_SLV0_DO	            0x06
#define B3_I2C_SLV1_ADDR	          0x07
#define B3_I2C_SLV1_REG	            0x08
#define B3_I2C_SLV1_CTRL	          0x09
#define B3_I2C_SLV1_DO	            0x0A
#define B3_I2C_SLV2_ADDR	          0x0B
#define B3_I2C_SLV2_REG	            0x0C
#define B3_I2C_SLV2_CTRL	          0x0D
#define B3_I2C_SLV2_DO	            0x0E
#define B3_I2C_SLV3_ADDR	          0x0F
#define B3_I2C_SLV3_REG	            0x10
#define B3_I2C_SLV3_CTRL	          0x11
#define B3_I2C_SLV3_DO	            0x12
#define B3_I2C_SLV4_ADDR	          0x13
#define B3_I2C_SLV4_REG	            0x14
#define B3_I2C_SLV4_CTRL	          0x15
#define B3_I2C_SLV4_DO	            0x16
#define B3_I2C_SLV4_DI	            0x17

#define SLAVE_ADDR                  0b1101000

// clang-format on

// Base
class ICM_20948
{
private:

  const uint8_t MAX_MAGNETOMETER_STARTS = 10; // This replaces maxTries

protected:
  ICM_20948_Device_t _device;

  float getTempC(int16_t val);
  float getGyrDPS(int16_t axis_val);
  float getAccMG(int16_t axis_val);
  float getMagUT(int16_t axis_val);

public:
  ICM_20948(); // Constructor

  void debugPrintStatus(ICM_20948_Status_e stat);

  ICM_20948_AGMT_t agmt;          // Acceleometer, Gyroscope, Magenetometer, and Temperature data
  ICM_20948_AGMT_t getAGMT(void); // Updates the agmt field in the object and also returns a copy directly

  float magX(void); // micro teslas
  float magY(void); // micro teslas
  float magZ(void); // micro teslas

  float accX(void); // milli g's
  float accY(void); // milli g's
  float accZ(void); // milli g's

  float gyrX(void); // degrees per second
  float gyrY(void); // degrees per second
  float gyrZ(void); // degrees per second

  float temp(void); // degrees celsius

  ICM_20948_Status_e status;                                              // Status from latest operation
  const char *statusString(ICM_20948_Status_e stat = ICM_20948_Stat_NUM); // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied

  // Device Level
  ICM_20948_Status_e setBank(uint8_t bank);                                // Sets the bank
  ICM_20948_Status_e swReset(void);                                        // Performs a SW reset
  ICM_20948_Status_e sleep(bool on = false);                               // Set sleep mode for the chip
  ICM_20948_Status_e lowPower(bool on = true);                             // Set low power mode for the chip
  ICM_20948_Status_e setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
  ICM_20948_Status_e checkID(void);                                        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

  bool dataReady(void);    // Returns 'true' if data is ready
  uint8_t getWhoAmI(void); // Return whoami in out prarmeter
  bool isConnected(void);  // Returns true if communications with the device are sucessful

  // Internal Sensor Options
  ICM_20948_Status_e setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
  ICM_20948_Status_e setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss);
  ICM_20948_Status_e setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg);
  ICM_20948_Status_e enableDLPF(uint8_t sensor_id_bm, bool enable);
  ICM_20948_Status_e setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt);

  // Interrupts on INT and FSYNC Pins
  ICM_20948_Status_e clearInterrupts(void);

  ICM_20948_Status_e cfgIntActiveLow(bool active_low);
  ICM_20948_Status_e cfgIntOpenDrain(bool open_drain);
  ICM_20948_Status_e cfgIntLatch(bool latching);         // If not latching then the interrupt is a 50 us pulse
  ICM_20948_Status_e cfgIntAnyReadToClear(bool enabled); // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
  ICM_20948_Status_e cfgFsyncActiveLow(bool active_low);
  ICM_20948_Status_e cfgFsyncIntMode(bool interrupt_mode); // Can use FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

  ICM_20948_Status_e intEnableI2C(bool enable);
  ICM_20948_Status_e intEnableDMP(bool enable);
  ICM_20948_Status_e intEnablePLL(bool enable);
  ICM_20948_Status_e intEnableWOM(bool enable);
  ICM_20948_Status_e intEnableWOF(bool enable);
  ICM_20948_Status_e intEnableRawDataReady(bool enable);
  ICM_20948_Status_e intEnableOverflowFIFO(uint8_t bm_enable);
  ICM_20948_Status_e intEnableWatermarkFIFO(uint8_t bm_enable);

  ICM_20948_Status_e WOMLogic(uint8_t enable, uint8_t mode);
  ICM_20948_Status_e WOMThreshold(uint8_t threshold);

  // Interface Options
  ICM_20948_Status_e i2cMasterPassthrough(bool passthrough = true);
  ICM_20948_Status_e i2cMasterEnable(bool enable = true);
  ICM_20948_Status_e i2cMasterReset();

  //Used for configuring peripherals 0-3
  ICM_20948_Status_e i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false, uint8_t dataOut = 0);
  ICM_20948_Status_e i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

  //Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
  //https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
  ICM_20948_Status_e i2cMasterConfigureSlave(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false);
  ICM_20948_Status_e i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

  //Used for configuring the Magnetometer
  ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data);
  uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg);

  // Default Setup
  ICM_20948_Status_e startupDefault(bool minimal = false); // If minimal is true, several startup steps are skipped. If ICM_20948_USE_DMP is defined, .begin will call startupDefault with minimal set to true.

  // direct read/write
  ICM_20948_Status_e read(uint8_t reg, uint8_t *pdata, uint32_t len);
  ICM_20948_Status_e write(uint8_t reg, uint8_t *pdata, uint32_t len);

  //Mag specific
  ICM_20948_Status_e startupMagnetometer(bool minimal = false); // If minimal is true, several startup steps are skipped. The mag then needs to be set up manually for the DMP.
  ICM_20948_Status_e magWhoIAm(void);
  uint8_t readMag(AK09916_Reg_Addr_e reg);
  ICM_20948_Status_e writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata);
  ICM_20948_Status_e resetMag();

  //FIFO
  ICM_20948_Status_e enableFIFO(bool enable = true);
  ICM_20948_Status_e resetFIFO(void);
  ICM_20948_Status_e setFIFOmode(bool snapshot = false); // Default to Stream (non-Snapshot) mode
  ICM_20948_Status_e getFIFOcount(uint16_t *count);
  ICM_20948_Status_e readFIFO(uint8_t *data, uint8_t len = 1);

  //DMP

  //Gyro Bias
  ICM_20948_Status_e setBiasGyroX(int32_t newValue);
  ICM_20948_Status_e setBiasGyroY(int32_t newValue);
  ICM_20948_Status_e setBiasGyroZ(int32_t newValue);
  ICM_20948_Status_e getBiasGyroX(int32_t* bias);
  ICM_20948_Status_e getBiasGyroY(int32_t* bias);
  ICM_20948_Status_e getBiasGyroZ(int32_t* bias);
  //Accel Bias
  ICM_20948_Status_e setBiasAccelX(int32_t newValue);
  ICM_20948_Status_e setBiasAccelY(int32_t newValue);
  ICM_20948_Status_e setBiasAccelZ(int32_t newValue);
  ICM_20948_Status_e getBiasAccelX(int32_t* bias);
  ICM_20948_Status_e getBiasAccelY(int32_t* bias);
  ICM_20948_Status_e getBiasAccelZ(int32_t* bias);
  //CPass Bias
  ICM_20948_Status_e setBiasCPassX(int32_t newValue);
  ICM_20948_Status_e setBiasCPassY(int32_t newValue);
  ICM_20948_Status_e setBiasCPassZ(int32_t newValue);
  ICM_20948_Status_e getBiasCPassX(int32_t* bias);
  ICM_20948_Status_e getBiasCPassY(int32_t* bias);
  ICM_20948_Status_e getBiasCPassZ(int32_t* bias);

  // Done:
  //  Configure DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  //  Load Firmware
  //  Configure Accel scaling to DMP
  //  Configure Compass mount matrix and scale to DMP
  //  Reset FIFO
  //  Reset DMP
  //  Enable DMP interrupt
  //  Configuring DMP to output data to FIFO: set DATA_OUT_CTL1, DATA_OUT_CTL2, DATA_INTR_CTL and MOTION_EVENT_CTL
  //  Configuring DMP to output data at multiple ODRs
  //  Configure DATA_RDY_STATUS
  //  Configuring Accel calibration
  //  Configuring Compass calibration
  //  Configuring Gyro gain
  //  Configuring Accel gain
  //  Configure I2C_SLV0 and I2C_SLV1 to: request mag data from the hidden reserved AK09916 registers; trigger Single Measurements
  //  Configure I2C Master ODR (default to 68.75Hz)

  // To Do:
  //  Additional FIFO output control: FIFO_WATERMARK, BM_BATCH_MASK, BM_BATCH_CNTR, BM_BATCH_THLD
  //  Configuring DMP features: PED_STD_STEPCTR, PED_STD_TIMECTR
  //  Enabling Activity Recognition (BAC) feature
  //  Enabling Significant Motion Detect (SMD) feature
  //  Enabling Tilt Detector feature
  //  Enabling Pick Up Gesture feature
  //  Enabling Fsync detection feature
  //  Biases: add save and load methods

  ICM_20948_Status_e enableDMP(bool enable = true);
  ICM_20948_Status_e resetDMP(void);
  ICM_20948_Status_e loadDMPFirmware(void);
  ICM_20948_Status_e setDMPstartAddress(unsigned short address = DMP_START_ADDRESS);
  ICM_20948_Status_e enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable = true);
  ICM_20948_Status_e enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable = true);
  ICM_20948_Status_e writeDMPmems(unsigned short reg, unsigned int length, const unsigned char *data);
  ICM_20948_Status_e readDMPmems(unsigned short reg, unsigned int length, unsigned char *data);
  ICM_20948_Status_e setDMPODRrate(enum DMP_ODR_Registers odr_reg, int interval);
  ICM_20948_Status_e readDMPdataFromFIFO(icm_20948_DMP_data_t *data);
  ICM_20948_Status_e setGyroSF(unsigned char div, int gyro_level);
  ICM_20948_Status_e initializeDMP(void) __attribute__((weak)); // Combine all of the DMP start-up code in one place. Can be overwritten if required
};

struct i2c_wires {
  int sda; // data
  int scl; // clock
};

// I2C

class ICM_20948_I2C : public ICM_20948
{
private:
protected:
public:
  struct i2c_wires _i2c;
  uint8_t _addr;
  uint8_t _ad0;
  bool _ad0val;
  ICM_20948_Serif_t _serif;

  ICM_20948_I2C(); // Constructor

  virtual ICM_20948_Status_e begin(struct i2c_wires wirePorts, bool ad0val = true, uint8_t ad0pin = ICM_20948_ARD_UNUSED_PIN);
};

#endif
