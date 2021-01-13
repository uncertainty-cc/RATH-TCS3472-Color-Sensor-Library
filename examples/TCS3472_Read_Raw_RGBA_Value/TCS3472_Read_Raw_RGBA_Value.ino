/**
 * @file TCS3472_Read_Raw_RGBA_Value.ino
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief In this example we will use the TCS3472 module to read raw RGBA values and process them.
 * @version 0.1
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
  Serial.println(" ==== TCS3472 Read Raw RGBA Value Demo ====");

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

  /* turn on the onboard LED */
  m_sensor.setLED(true);          /* you can also try to turn OFF the LED by changing `true` to `false` */
}

void loop() {
  /* creates a 4 * 2byte array to store values */
  uint16_t data[4];

  /* reads the sensor output */
  m_sensor.get(data);

  /* the format of the data array will now be: [R, G, B, Alpha] */
  
  /* some calculations for better visualization */
  uint32_t alpha = data[3];
  float r, g, b;
  r = data[0];
  r /= alpha;
  g = data[1];
  g /= alpha;
  b = data[2];
  b /= alpha;

  /* print out the values */
  Serial.print("R: ");
  Serial.print(r);
  Serial.print("\tG: "); 
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
  Serial.print("\tAlpha: ");
  Serial.print(alpha);
  Serial.println();
}
