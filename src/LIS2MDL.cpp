#include "LIS2MDL.h"

LIS2MDL::LIS2MDL(uint8_t comm, uint8_t inputAddress) {
  commMode = comm;
  address = inputAddress;
  
  // default settings
  settings.tempCompensationEnabled = LIS2MDL_TEMP_COMPENSATION_ENABLED;
  settings.rebootMode = LIS2MDL_REBOOT_NORMAL_MODE;
  settings.resetMode = LIS2MDL_NO_RESET_MODE;
  settings.powerMode = LIS2MDL_POWERMODE_HIGH;
  settings.magSampleRate = LIS2MDL_MAG_ODR_10Hz;
  settings.operationMode = LIS2MDL_CONTINUOUS_MODE;

  settings.singleModeOffsetCancellationEnabled = LIS2MDL_SINGLE_MODE_OFF_CANC_DISABLED;
  settings.checkDataAfterHardIronCorrectionEnabled = LIS2MDL_HARD_IRON_CORRECTION_NO_CHECK;
  settings.setPulseFrequency = LIS2MDL_RELEASE_EVERY_63_ODR;
  settings.offsetCancellationEnabled = LIS2MDL_OFFSET_CANCELLATION_DISABLED;
  settings.lowPassFilterEnabled = LIS2MDL_LOW_PASS_FILTER_ENABLED;

  settings.interruptEnabled = LIS2MDL_INTERRUPT_DISABLED;
  settings.i2cDisabled = LIS2MDL_I2C_ENABLED;
  settings.readSafety = LIS2MDL_SAFE_ASYNC_READ;
  settings.endianness = LIS2MDL_BIG_ENDIAN;
  settings.spiConfig = LIS2MDL_4WIRESPI_DISABLED;
  settings.selfTestEnabled = LIS2MDL_SELFTEST_DISABLED;
  settings.dataReadyEnabled = LIS2MDL_DATA_READY_ENABLED;

  settings.magSensitivity = 1.5f;

  allOnesCounter = 0;
  nonSuccessCounter = 0;
}

mag_status_t LIS2MDL::begin(bool bypassWireInit) {
  mag_status_t status = wireUp(bypassWireInit);

  // don't go further if it failed to start up
  return status;
}

mag_status_t LIS2MDL::wireUp(bool bypassWireInit) {
  mag_status_t result = MAG_SUCCESS;

  if (!bypassWireInit) {
    switch(commMode) {
      case I2C_MODE:
        pinMode(21,INPUT_PULLUP);
        pinMode(22,INPUT_PULLUP);
        Wire.begin(21, 22, 1 * 1000 * 1000);
        break;
      case SPI_MODE:
        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV16);
        SPI.setBitOrder(MSBFIRST);

        pinMode(address, OUTPUT);
        digitalWrite(address, HIGH);
        break;
    }
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
  return readInt16(LIS2MDL_OUTX_L_REG);
}

int16_t LIS2MDL::readRawMagY() {
  return readInt16(LIS2MDL_OUTY_L_REG);
}

int16_t LIS2MDL::readRawMagZ() {
  return readInt16(LIS2MDL_OUTZ_L_REG);
}

float LIS2MDL::readFloatMagX() {
  return (float)readRawMagX() * settings.magSensitivity;
}

float LIS2MDL::readFloatMagY() {
  return (float)readRawMagY() * settings.magSensitivity;
}

float LIS2MDL::readFloatMagZ() {
  return (float)readRawMagZ() * settings.magSensitivity;
}

int16_t LIS2MDL::readRawTemp() {
  return readInt16(LIS2MDL_TEMP_OUT_L_REG);
}

float LIS2MDL::readTempC() {
  return (float)readRawTemp() / 8.0f + 25.0f;
}

float LIS2MDL::readTempF() {
  return (readTempC() * 9.0f / 5.0f) + 32.0f;
}

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
  configC |= settings.selfTestEnabled;
  configC |= settings.dataReadyEnabled;

  // write CFG_REG_A
  mag_status_t status = write(LIS2MDL_CFG_REG_A, configA);

  // write CFG_REG_B if A succeeded
  if (status == MAG_SUCCESS)
    status = write(LIS2MDL_CFG_REG_B, configB);

  // write CFG_REG_C if B succeeded
  if (status == MAG_SUCCESS)
    status = write(LIS2MDL_CFG_REG_C, configC);

  return status;
}

int16_t LIS2MDL::readInt16(lis2mdlRegisters_t offset) {
  uint8_t buffer[2];
  mag_status_t result = readRegion(offset, buffer, 2);
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
      Wire.beginTransmission(address);
      Wire.write(offset);
      if(Wire.endTransmission() != 0)
        status = MAG_HW_ERROR;
      else {
        Wire.requestFrom(address, length);
        while ((Wire.available()) && (i < length)) {
          c = Wire.read();
          *output = c;
          output++;
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
          allOnesCounter++;

        *output = c;
        output++;
        ESP_LOGI("Read", "Mag read: %d", c);
        i++;
      }
      if(allOnesCounter == i)
        status = MAG_ALL_ONES_WARNING;

      digitalWrite(address, HIGH);
      break;
  }

  return status;
}

mag_status_t LIS2MDL::write(uint8_t offset, uint8_t data) {
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

void LIS2MDL::reset() {
  uint8_t state;
  read(LIS2MDL_CFG_REG_A, &state);
  write(LIS2MDL_CFG_REG_A, state | LIS2MDL_SOFT_RESET);
  delay(1);
  write(LIS2MDL_CFG_REG_A, state | LIS2MDL_REBOOT_CLEAR_MEMORY);
  delay(100);
}