#include "LIS2MDL.h"

LIS2MDL::LIS2MDL(comm_mode_t comm, uint8_t inputAddress) {
  commMode = comm;
  address = inputAddress;
  
  // default settings
  settings.tempCompensationEnabled = LIS2MDL_TEMP_COMPENSATION_ENABLED;
  settings.rebootMode = LIS2MDL_REBOOT_NORMAL;
  settings.resetMode = LIS2MDL_NO_RESET;
  settings.powerMode = LIS2MDL_POWERMODE_HIGH;
  settings.magSampleRate = LIS2MDL_MAG_ODR_10Hz;
  settings.operationMode = LIS2MDL_CONTINUOUS_MODE;

  settings.singleModeOffsetCancellationEnabled = 0;
  settings.checkDataAfterHardIronCorrectionEnabled = LIS2MDL_HARD_IRON_CORRECTION_CHECK;
  settings.setPulseFrequency = 0;
  settings.offsetCancellationEnabled = 0;
  settings.lowPassFilterEnabled = LIS2MDL_LOW_PASS_FILTER_ENABLED;

  settings.interruptEnabled = 0;
  settings.i2cDisabled = commMode == SPI_MODE ? LIS2MDL_I2C_DISABLED : LIS2MDL_I2C_ENABLED;
  settings.readSafety = LIS2MDL_SAFE_ASYNC_READ;
  settings.endianness = LIS2MDL_BIG_ENDIAN;
  settings.spiConfig = LIS2MDL_4WIRESPI_DISABLED;
  settings.dataReadyEnabled = 0;

  settings.magSensitivity = 0.0015f;

  allOnesCounter = 0;
  nonSuccessCounter = 0;
}

mag_status_t LIS2MDL::begin() {
  mag_status_t status = wireUp();

  // don't go further if it failed to start up
  if (status != MAG_SUCCESS)
    return status;

  // write settings to device
  writeSettings();
}

mag_status_t LIS2MDL::wireUp() {
  mag_status_t result = MAG_SUCCESS;
  switch(commMode) {
    case I2C_MODE:
      Wire.begin();
      break;
    case SPI_MODE:
      SPI.begin();
      SPI.setClockDivider(SPI_CLOCK_DIV4);
      SPI.setBitOrder(MSBFIRST);

      pinMode(chipSelect, OUTPUT);
      digitalWrite(chipSelect, HIGH);
      break;
  }

  //Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ )
	{
		temp++;
	}

  uint8_t readCheck;
  read(LIS2MDL_WHO_AM_I, &readCheck);
  if (readCheck != deviceId) 
    result = MAG_HW_ERROR;

  return result;
}

int16_t LIS2MDL::readRawMagX() {
  return readInt16(LIS2MDL_OUT_X_REG_L);
}

int16_t LIS2MDL::readRawMagY() {
  return readInt16(LIS2MDL_OUT_Y_REG_L);
}

int16_t LIS2MDL::readRawMagZ() {
  return readInt16(LIS2MDL_OUT_Z_REG_L);
}

float LIS2MDL::readFloatMagX() {
  // maxBiasX and magScaleX must be set for this to work*
  return ((float)readRawMagX() * settings.magSensitivity - magBiasX) * magScaleX;
}

float LIS2MDL::readFloatMagY() {
  // maxBiasY and magScaleY must be set for this to work*
  return ((float)readRawMagY() * settings.magSensitivity - magBiasY) * magScaleY;
}

float LIS2MDL::readFloatMagZ() {
  // maxBiasZ and magScaleZ must be set for this to work
  return ((float)readRawMagZ() * settings.magSensitivity - magBiasZ) * magScaleZ;
}

int16_t readRawTemp();
float readTempC();
float readTempF();

mag_status_t LIS2MDL::writeSettings() {
  uint8_t configA = 0;
  uint8_t configB = 0;
  uint8_t configC = 0;

  // set data for register A
  configA |= settings.tempCompensationEnabled;
  configA |= settings.rebootMode;
  configA |= settings.resetMode;
  configA |= settings.powerMode;
  configA |= settings.magSampleRate;
  configA |= settings.operationMode;

  // set data for register B
  configB |= settings.singleModeOffsetCancellationEnabled;
  configB |= settings.checkDataAfterHardIronCorrectionEnabled;
  configB |= settings.setPulseFrequency;
  configB |= settings.offsetCancellationEnabled;
  configB |= settings.lowPassFilterEnabled;

  // set data for register C
  configC |= settings.interruptEnabled;
  configC |= settings.i2cDisabled;
  configC |= settings.readSafety;
  configC |= settings.endianness;
  configC |= settings.spiConfig;
  configC |= selfTestEnabled;
  configC |= dataReadyEnabled;

  // write all registers
  status = writeRegister(LIS2MDL_CFG_REG_A, configA);
  if (status == MAG_SUCCESS)
    status = writeRegister(LIS2MDL_CFG_REG_A, configB);
  if (status == MAG_SUCCESS)
    status = writeRegister(LIS2MDL_CFG_REG_A, configC);

  return status;
}

int16_t LIS2MDL::readInt16(lis2mdlRegisters_t offset) {
  uint8_t buffer[2];
  status_t result = readRegion(offset, buffer, 2);
  int16_t output = (int16_t)buffer[0] | int16_t(buffer[1] << 8);

  if (result != MAG_SUCCESS) {
    if (result == MAG_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }

  return output;
}

mag_status_t LIS2MDL::read(lis2mdlRegisters_t offset, uint8_t *output) {
  return readRegion(offset, output, 1);
}

mag_status_t LIS2MDL::readRegion(lis2mdlRegisters_t offset, uint8_t *output, uint8_t length) {
  mag_status_t status = MAG_SUCCESS;
  uint8_t i = 0;
  uint8_t c = 0;

  switch (commMode) {
    case I2C_MODE:
      Wire.beginTransmission(I2CAddress);
      Wire.write(offset);
      if(Wire.endTransmission() != 0)
        status = IMU_HW_ERROR;
      else {
        Wire.requestFrom(I2CAddress, length);
        while ((Wire.available()) && (i < length)) {
          c = Wire.read();
          *outputPointer = c;
          outputPointer++;
          i++;
        }
      }
      break;
    case SPI_MODE:
      digitalWrite(address, LOW);

      SPI.transfer(offset | 0x80);
      while (i < length) {
        c = SPI.transfer(0x00);
        if(c == 0xFF)
          tempFFCounter++;

        *outputPointer = c;
        outputPointer++;
        i++;
      }
      if(tempFFCounter == i)
        status = IMU_ALL_ONES_WARNING;

      digitalWrite(chipSelectPin, HIGH);
      break;
  }
}

mag_status_t LIS2MDL::writeRegister(uint8_t offset, uint8_t data) {
  mag_status_t status = MAG_SUCCESS;
  switch (commMode) {
    case I2C_MODE:
      Wire.beginTransmission(address);
      Wire.write(offset);
      Wire.write(data);
      if (Wire.endTransmission() != 0)
        status = MAG_HW_ERROR;
      break;
    case SPI_MODE:
      digitalWrite(address, LOW);
      SPI.transfer(offset);
      SPI.transfer(data);
      digitalWrite(address, HIGH);
      break;
    default:
      break;
  }
  return status;
}