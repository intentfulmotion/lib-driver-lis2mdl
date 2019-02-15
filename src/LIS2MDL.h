#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define VERSION 1

typedef enum
{
	MAG_SUCCESS,
	MAG_HW_ERROR,
	MAG_NOT_SUPPORTED,
	MAG_GENERIC_ERROR,
	MAG_OUT_OF_BOUNDS,
	MAG_ALL_ONES_WARNING,
} mag_status_t;

typedef enum {
  I2C_MODE,
  SPI_MODE
} comm_mode_t;

typedef enum {
  // hard-iron registers
  LIS2MDL_OFFSET_X_REG_L                = 0x45,
  LIS2MDL_OFFSET_X_REG_H                = 0x46,
  LIS2MDL_OFFSET_Y_REG_L                = 0x47,
  LIS2MDL_OFFSET_Y_REG_H                = 0x48,
  LIS2MDL_OFFSET_Z_REG_L                = 0x49,
  LIS2MDL_OFFSET_Z_REG_H                = 0x50,

  // identity register              
  LIS2MDL_WHO_AM_I                      = 0x4F,

  // coniguration register              s
  LIS2MDL_CFG_REG_A                     = 0x60,
  LIS2MDL_CFG_REG_B                     = 0x61,
  LIS2MDL_CFG_REG_C                     = 0x62,

  // interrupt registers              
  LIS2MDL_INT_CTRL_REG                  = 0x63,
  LIS2MDL_INT_SOURCE_REG                = 0x64,
  LIS2MDL_INT_THS_L_REG                 = 0x65,
  LIS2MDL_INT_THS_H_REG                 = 0x66,

  // status register              
  LIS2MDL_STATUS_REG                    = 0x67,

  // output registers             
  LIS2MDL_OUTX_L_REG                    = 0x68,
  LIS2MDL_OUTX_H_REG                    = 0x69,
  LIS2MDL_OUTY_L_REG                    = 0x6A,
  LIS2MDL_OUTY_H_REG                    = 0x6B,
  LIS2MDL_OUTZ_L_REG                    = 0x6C,
  LIS2MDL_OUTZ_H_REG                    = 0x6D,

  // temperature sensor registers
  LIS2MDL_TEMP_OUT_L_REG                = 0x6E,
  LIS2MDL_TEMP_OUT_H_REG                = 0x6F
} lis2mdlRegisters_t;

// CFG_REG_A bits
#define LIS2MDL_REBOOT_CLEAR_MEMORY       0x40
#define LIS2MDL_SOFT_RESET                0x20
#define LIS2MDL_LOW_POWER_MODE            0x10

// CFG_REG_B bits
#define LIS2MDL_OFF_CANC_ONE_SHOT         0x10
#define LIS2MDL_INT_ON_DATA_OFF           0x08
#define LIS2MDL_SET_FREQUENCY             0x04
#define LIS2MDL_OFFSET_CANCELLATION       0x02
#define LIS2MDL_LOW_PASS_FILTER           0x01

// CFG_REG_C bits
#define LIS2MDL_INT_ON_PIN                0x40
#define LIS2MDL_I2C_DISABLE               0x20
#define LIS2MDL_DATA_SYNC                 0x10
#define LIS2MDL_ENDIAN_SWAP               0x08
#define LIS2MDL_4WSPI                     0x04
#define LIS2MDL_SELFTEST                  0x02
#define LIS2MDL_DRDY_ON_PIN               0x01

typedef enum {
  LIS2MDL_TEMP_COMPENSATION_DISABLED    = 0x00,
  LIS2MDL_TEMP_COMPENSATION_ENABLED     = 0x80
} lis2mdlTempCompensation;

typedef enum {
  LIS2MDL_REBOOT_NORMAL_MODE            = 0x00,
  LIS2MDL_REBOOT_CLEAR_MEMORY_MODE      = LIS2MDL_REBOOT_CLEAR_MEMORY
} lis2mdlRebootMode;

typedef enum {
  LIS2MDL_NO_RESET_MODE                 = 0x00,
  LIS2MDL_SOFT_RESET_MODE               = LIS2MDL_SOFT_RESET
} lis2mdlResetMode;

typedef enum {
  LIS2MDL_POWERMODE_HIGH                = 0x00,
  LIS2MDL_POWERMODE_LOW                 = LIS2MDL_LOW_POWER_MODE
} lis2mdlPowerMode;

typedef enum {
  LIS2MDL_MAG_ODR_10Hz                  = 0x00,   // 10 Hz
  LIS2MDL_MAG_ODR_20Hz                  = 0x01,   // 20 Hz
  LIS2MDL_MAG_ODR_50Hz                  = 0x02,   // 50 Hz
  LIS2MDL_MAG_ODR_100Hz                 = 0x03    // 100 Hz
} lis2mdlOutputDataRate;

typedef enum {
  LIS2MDL_CONTINUOUS_MODE               = 0x00,
  LIS2MDL_SINGLE_MODE                   = 0x01,
  LIS2MDL_IDLE_MODE                     = 0x02,
  LIS2MDL_IDLE_MODE_1                   = 0x03
} lis2mdlOperationMode;

typedef enum {
  LIS2MDL_SINGLE_MODE_OFF_CANC_DISABLED = 0x00,
  LIS2MDL_SINGLE_MODE_OFF_CANC_ENABLED  = LIS2MDL_OFF_CANC_ONE_SHOT
} lis2mdlSingleModeOffsetCancellation;

typedef enum {
  LIS2MDL_HARD_IRON_CORRECTION_NO_CHECK = 0x00,
  LIS2MDL_HARD_IRON_CORRECTION_CHECK    = LIS2MDL_INT_ON_DATA_OFF
} lis2mdlHardIronCorrectionDataCheck;

typedef enum {
  LIS2MDL_RELEASE_EVERY_63_ODR          = 0x00,
  LIS2MDL_RELEASE_ONLY_POWER_ON         = LIS2MDL_SET_FREQUENCY
} lis2mdlFrequency;

typedef enum {
  LIS2MDL_OFFSET_CANCELLATION_DISABLED  = 0x00,
  LIS2MDL_OFFSET_CANCELLATION_ENABLED   = LIS2MDL_OFFSET_CANCELLATION
} lis2mdlOffsetCancellation;

typedef enum {
  LIS2MDL_LOW_PASS_FILTER_DISABLED      = 0x00,
  LIS2MDL_LOW_PASS_FILTER_ENABLED       = LIS2MDL_LOW_PASS_FILTER
} lis2mdlLowPassFilter;

typedef enum {
  LIS2MDL_INTERRUPT_DISABLED            = 0x00,
  LIS2MDL_INTERRUPT_ENABLED             = LIS2MDL_INT_ON_PIN
} lis2mdlInterrupt;

typedef enum {
  LIS2MDL_I2C_ENABLED                   = 0x00,
  LIS2MDL_I2C_DISABLED                  = LIS2MDL_I2C_DISABLE
} lis2mdlInterface;

typedef enum {
  LIS2MDL_UNSAFE_ASYNC_READ             = 0x00,
  LIS2MDL_SAFE_ASYNC_READ               = LIS2MDL_DATA_SYNC
} lis2mdlReadSafety;

typedef enum {
  LIS2MDL_BIG_ENDIAN                    = 0x00,
  LIS2MDL_LITTLE_ENDIAN                 = LIS2MDL_ENDIAN_SWAP
} lis2mdlEndian;

typedef enum {
  LIS2MDL_4WIRESPI_DISABLED             = 0x00,
  LIS2MDL_4WIRESPI_ENABLED              = LIS2MDL_4WSPI
} lis2mdlSPIConfig;

typedef enum {
  LIS2MDL_SELFTEST_DISABLED             = 0x00,
  LIS2MDL_SELFTEST_ENABLED              = LIS2MDL_SELFTEST
} lis2mdlSelfTest;

typedef enum {
  LIS2MDL_DATA_READY_DISABLED           = 0x00,
  LIS2MDL_DATA_READY_ENABLED            = LIS2MDL_DRDY_ON_PIN
} lis2mdlDataReady;

// TODO: add paramters for interrupt configuration

struct MagSensorSettings {
  public:
    comm_mode_t                         commMode;
    uint8_t                             address;
    lis2mdlTempCompensation             tempCompensationEnabled;
    lis2mdlRebootMode                   rebootMode;
    lis2mdlResetMode                    resetMode;
    lis2mdlPowerMode                    powerMode;
    lis2mdlOutputDataRate               magSampleRate;
    lis2mdlOperationMode                operationMode;

    lis2mdlSingleModeOffsetCancellation singleModeOffsetCancellationEnabled;
    lis2mdlHardIronCorrectionDataCheck  checkDataAfterHardIronCorrectionEnabled;
    lis2mdlFrequency                    setPulseFrequency;
    lis2mdlOffsetCancellation           offsetCancellationEnabled;
    lis2mdlLowPassFilter                lowPassFilterEnabled;

    lis2mdlInterrupt                    interruptEnabled;
    lis2mdlInterface                    i2cDisabled;
    lis2mdlReadSafety                   readSafety;
    lis2mdlEndian                       endianness;
    lis2mdlSPIConfig                    spiConfig;
    lis2mdlSelfTest                     selfTestEnabled;
    lis2mdlDataReady                    dataReadyEnabled;

    float                               magBiasX;
    float                               magBiasY;
    float                               magBiasZ;

    float                               magScaleX;
    float                               magScaleY;
    float                               magScaleZ;
    float                               magSensitivity;
};

class LIS2MDL {
  uint8_t deviceId = 0x40;              // 01000000b
  comm_mode_t commMode;
  uint8_t address;                      // chip select for SPI, I2C address for I2C

  mag_status_t wireUp();

  int16_t readInt16(lis2mdlRegisters_t offset);

  mag_status_t read(lis2mdlRegisters_t offset, uint8_t *output);
  mag_status_t readRegion(lis2mdlRegisters_t offset, uint8_t *output, uint8_t length);
  mag_status_t write(uint8_t offset, uint8_t data);

  public:
    MagSensorSettings settings;

    LIS2MDL(comm_mode_t comm = SPI_MODE, uint8_t inputAddress = 5);
    ~LIS2MDL() = default;

    uint16_t allOnesCounter;
    uint16_t nonSuccessCounter;

    mag_status_t begin();
    mag_status_t writeSettings();
    void calibrate(uint32_t reads = 4000);

    int16_t readRawMagX();
    int16_t readRawMagY();
    int16_t readRawMagZ();

    float readFloatMagX();
    float readFloatMagY();
    float readFloatMagZ();

    int16_t readRawTemp();
    float readTempC();
    float readTempF();
}