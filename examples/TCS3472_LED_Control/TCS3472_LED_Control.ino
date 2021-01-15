/**
 * @file TCS3472_LED_Control.ino
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief In this example we will toggle the onboard LED with a frequency of 0.5 Hz (cycle between 1s ON and 1s OFF)
 * @version 0.2
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* ======================================= *
  Wiring Diagram

  | Arduino  | TCS3472 |
  | -------- | ------- |
  | GND      | GND     |
  | 3V3      | 3V3     |
  | A5 (SCL) | SCL     |
  | A4 (SDA) | SDA     |

 * ======================================= */

#include "RATH_TCS3472.h"

/* creates a TCS3472 object */
Rath::TCS3472 m_sensor;

void setup() {
  /* initialize serial monitor */
  Serial.begin(115200);
  Serial.println(" ==== TCS3472 LED Control Demo ====");

  /* initialize the sensor */
  m_sensor.init();

  /* test if the sensor is connected */
  if (!m_sensor.isConnected()) {
    while (1) {
      /* if not connected, there is no purpose to proceed */
    }
  }

  /* by default, the device is in power down mode on bootup, so we need to enable it */
  /* this function will also set the default gain and integration time */
  m_sensor.enable();
}

void loop() {
  /* turn on the onboard LED */
  m_sensor.setLED(true);

  delay(1000);
  
  /* turn off the onboard LED */
  m_sensor.setLED(false);

  delay(1000);
}
