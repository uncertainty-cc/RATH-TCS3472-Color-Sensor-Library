/**
 * @file RATH_TCS3472.cpp
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.2
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "RATH_TCS3472.h"

using namespace Rath;


#ifdef ARDUINO
void TCS3472::readMemory(uint8_t mem_address, uint8_t *buffer, uint16_t size) {
  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | mem_address);
  Wire.endTransmission();

  uint16_t i = 0;
  
  Wire.requestFrom(TCS3472_I2C_ADDRESS, size);
  while (size > 0) {
    *buffer = Wire.read();
    size -= 1;
    buffer += sizeof(uint8_t);
  }
}

void TCS3472::writeMemory(uint8_t mem_address, uint8_t *buffer, uint16_t size) {
  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | mem_address);
  
  while (size > 0) {
    Wire.write(*buffer);
    size -= 1;
    buffer += sizeof(uint8_t);
  }

  Wire.endTransmission();
}

void TCS3472::init() {
  Wire.begin();
}


void TCS3472::setLED(uint8_t state) {
  /* clear interrupt */
  Wire.beginTransmission(TCS3472_I2C_ADDRESS);
  Wire.write(TCS3472_COMMAND_BIT | 0x66);
  Wire.endTransmission();

  uint8_t data;
  readMemory(TCS3472_ENABLE_REG, &data, sizeof(uint8_t));
  if (state) {
    data &= ~TCS3472_ENABLE_AIEN_MSK;
  }
  else {
    data |= TCS3472_ENABLE_AIEN_MSK;
  }
  writeMemory(TCS3472_ENABLE_REG, &data, sizeof(uint8_t));
}

void TCS3472::delay(uint32_t time) {
  delay(time);
}
#endif
#ifdef PLATFORM_TOKISAKI_HAL

void TCS3472::readMemory(uint8_t mem_address, uint8_t *data, uint16_t size) {
  uint8_t buffer;
  buffer = TCS3472_COMMAND_BIT | mem_address;
  HAL_I2C_masterTransmit(I2C1, TCS3472_I2C_ADDRESS, &buffer, sizeof(uint8_t), timeout);

  HAL_I2C_masterReceive(I2C1, TCS3472_I2C_ADDRESS, data, size, timeout);
}

void TCS3472::writeMemory(uint8_t mem_address, uint8_t *data, uint16_t size) {
  uint8_t buffer[size + sizeof(uint8_t)];
  
  buffer[0] = TCS3472_COMMAND_BIT | mem_address;
  for (uint8_t i=0; i<size; i+=1) {
    buffer[i+1] = data[i];
  }

  HAL_I2C_masterTransmit(I2C1, TCS3472_I2C_ADDRESS, buffer, size + sizeof(uint8_t), timeout);
}

void TCS3472::init() {
  
}


void TCS3472::setLED(uint8_t state) {
  /* clear interrupt */
  uint8_t buffer;
  
  buffer = TCS3472_COMMAND_BIT | 0x66;

  HAL_I2C_masterTransmit(I2C1, TCS3472_I2C_ADDRESS, &buffer, sizeof(uint8_t), timeout);
  
  readMemory(TCS3472_ENABLE_REG, &buffer, sizeof(uint8_t));
  if (state) {
    buffer &= ~TCS3472_ENABLE_AIEN_MSK;
  }
  else {
    buffer |= TCS3472_ENABLE_AIEN_MSK;
  }
  writeMemory(TCS3472_ENABLE_REG, &buffer, sizeof(uint8_t));
}

void TCS3472::delay(uint32_t time) {
  HAL_delay(time);
}
#endif

void TCS3472::enable() {
  uint8_t data;
  data = TCS3472_ENABLE_PON_MSK;
  writeMemory(TCS3472_ENABLE_REG, &data, sizeof(uint8_t));

  /* delay for an amount of time to let the module power up */
  delay(50);
  data = TCS3472_ENABLE_PON_MSK | TCS3472_ENABLE_AEN_MSK | TCS3472_ENABLE_AIEN_MSK;
  writeMemory(TCS3472_ENABLE_REG, &data, sizeof(uint8_t));

  setIntegrationTime(intergration_time);
  setGain(gain);
}

uint8_t TCS3472::isConnected() {
  uint8_t data;

  readMemory(TCS3472_ID_REG, &data, sizeof(uint8_t));

  if ((data != TCS34721_ID_VALUE) && (data != TCS34725_ID_VALUE)
    && (data != TCS34723_ID_VALUE) && (data != TCS34727_ID_VALUE)) {
    return 0;
  }
  return data;
}

char* TCS3472::getDeviceName() {
  uint8_t x = isConnected();
  if (!x) {
    return (char *)"No Device Connected.";
  }
  switch (x) {
    case TCS34721_ID_VALUE:
      return (char *)"TCS34721 or TCS34725";
    case TCS34723_ID_VALUE:
      return (char *)"TCS34723 or TCS34727";
  }
  return (char *)"Unknown";
}

void TCS3472::setIntegrationTime(uint8_t value) {
  intergration_time = value;
  writeMemory(TCS3472_ATIME_REG, &intergration_time, sizeof(uint8_t));
}

void TCS3472::setGain(uint8_t value) {
  gain = value;
  writeMemory(TCS3472_CONTROL_REG, &gain, sizeof(uint8_t));
}

uint8_t TCS3472::getLED() {
  uint8_t data;
  readMemory(TCS3472_STATUS_REG, &data, sizeof(uint8_t));
  return (data & TCS3472_ENABLE_AIEN_MSK) ? 0 : 1;
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

  uint8_t data[2];
  
  readMemory(TCS3472_RDATAL_REG, data, 2);
  buffer[0] = data[1] << 8 | data[0];
  readMemory(TCS3472_GDATAL_REG, data, sizeof(uint16_t));
  buffer[1] = data[1] << 8 | data[0];
  readMemory(TCS3472_BDATAL_REG, data, sizeof(uint16_t));
  buffer[2] = data[1] << 8 | data[0];
  readMemory(TCS3472_CDATAL_REG, data, sizeof(uint16_t));
  buffer[3] = data[1] << 8 | data[0];
}

void TCS3472::get(uint16_t *R, uint16_t *G, uint16_t *B, uint16_t *alpha) {
  uint16_t buffer[4];
  get(buffer);
  
  *R = buffer[0];
  *G = buffer[1];
  *B = buffer[2];
  *alpha = buffer[3];
}