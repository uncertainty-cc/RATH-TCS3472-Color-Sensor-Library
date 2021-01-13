/**
 * @file RATH_TCS3472.cpp
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "RATH_TCS3472.h"

using namespace Rath;

uint8_t TCS3472::readReg(uint8_t address) {
  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | address);
  Wire.endTransmission();

  Wire.requestFrom(TCS3472_I2C_ADDRESS, 1);
  return Wire.read();
}

uint16_t TCS3472::readRegWord(uint8_t address) {
  uint16_t x; uint16_t t;

  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | address);
  Wire.endTransmission();

  Wire.requestFrom(TCS3472_I2C_ADDRESS, 2);
  t = Wire.read();
  x = Wire.read();
  x <<= 8;
  x |= t;
  return x;
}

void TCS3472::writeReg(uint8_t address, uint32_t value) {
  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | address);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

void TCS3472::init() {
  Wire.begin();
}

uint8_t TCS3472::enable() {
  writeReg(TCS3472_ENABLE_REG, TCS3472_ENABLE_PON_MSK);

  /* delay for an amount of time to let the module power up */
  delay(3);
  writeReg(TCS3472_ENABLE_REG, TCS3472_ENABLE_PON_MSK | TCS3472_ENABLE_AEN_MSK | TCS3472_ENABLE_AIEN_MSK);  

  setIntegrationTime(intergration_time);
  setGain(gain);
}

uint8_t TCS3472::isConnected() {
  uint8_t x = readReg(TCS3472_ID_REG);
  if ((x != TCS34721_ID_VALUE) && (x != TCS34725_ID_VALUE) && (x != TCS34723_ID_VALUE) && (x != TCS34727_ID_VALUE)) {
    return 0;
  }
  return x;
}

char* TCS3472::getDeviceName() {
  uint8_t x = isConnected();
  if (!x) {
    return "No Device Connected.";
  }
  switch (x) {
    case TCS34721_ID_VALUE:
      return "TCS34721 or TCS34725";
    case TCS34723_ID_VALUE:
      return "TCS34723 or TCS34727";
  }
  return "Unknown";
}

void TCS3472::setIntegrationTime(uint32_t value) {
  intergration_time = value;
  writeReg(TCS3472_ATIME_REG, value);
}

void TCS3472::setGain(uint32_t value) {
  writeReg(TCS3472_CONTROL_REG, value);
}

uint8_t TCS3472::getLED() {
  return (readReg(TCS3472_STATUS_REG) & TCS3472_ENABLE_AIEN_MSK) ? 0 : 1;
}

void TCS3472::setLED(uint8_t state) {
  /* clear interrupt */
  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | 0x66);
  Wire.endTransmission();

  uint8_t r = readReg(TCS3472_ENABLE_REG);
  if (state) {
    r &= ~TCS3472_ENABLE_AIEN_MSK;
  }
  else {
    r |= TCS3472_ENABLE_AIEN_MSK;
  }
  writeReg(TCS3472_ENABLE_REG, r);
}

void TCS3472::get(uint16_t *buffer) {
  /* Set a delay for the integration time */
  switch (intergration_time) {
  case TCS3472_INTEGRATIONTIME_2_4MS:
    delay(3);
    break;
  case TCS3472_INTEGRATIONTIME_24MS:
    delay(24);
    break;
  case TCS3472_INTEGRATIONTIME_50MS:
    delay(50);
    break;
  case TCS3472_INTEGRATIONTIME_101MS:
    delay(101);
    break;
  case TCS3472_INTEGRATIONTIME_154MS:
    delay(154);
    break;
  case TCS3472_INTEGRATIONTIME_700MS:
    delay(700);
    break;
  }
  
  buffer[0] = readRegWord(TCS3472_RDATAL_REG);
  buffer[1] = readRegWord(TCS3472_GDATAL_REG);
  buffer[2] = readRegWord(TCS3472_BDATAL_REG);
  buffer[3] = readRegWord(TCS3472_CDATAL_REG);
}

void TCS3472::get(uint16_t *R, uint16_t *G, uint16_t *B, uint16_t *alpha) {
  /* Set a delay for the integration time */
  switch (intergration_time) {
  case TCS3472_INTEGRATIONTIME_2_4MS:
    delay(3);
    break;
  case TCS3472_INTEGRATIONTIME_24MS:
    delay(24);
    break;
  case TCS3472_INTEGRATIONTIME_50MS:
    delay(50);
    break;
  case TCS3472_INTEGRATIONTIME_101MS:
    delay(101);
    break;
  case TCS3472_INTEGRATIONTIME_154MS:
    delay(154);
    break;
  case TCS3472_INTEGRATIONTIME_700MS:
    delay(700);
    break;
  }
  
  *R = readRegWord(TCS3472_RDATAL_REG);
  *G = readRegWord(TCS3472_GDATAL_REG);
  *B = readRegWord(TCS3472_BDATAL_REG);
  *alpha = readRegWord(TCS3472_CDATAL_REG);
}
