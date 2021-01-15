/**
 * @file RATH_TCS3472.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief
 * @version 0.2
 * @date 2021-01-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __RATH_TCS3472_H  // include guard
#define __RATH_TCS3472_H

#include <stdint.h>
#include <string.h>

#ifdef ARDUINO

#include <Arduino.h>
#include <Wire.h>

#endif

#ifdef PLATFORM_TOKISAKI_HAL

#include "gd32vf1xx_hal.h"

#endif

#define TCS3472_I2C_ADDRESS                     0x29

#define TCS3472_COMMAND_BIT                     0x80U

/** Enable Register */
#define TCS3472_ENABLE_REG                      0x00U
#define TCS3472_ENABLE_PON_POS                  0x0U  /** Power ON. This bit activates the internal oscillator to permit the timers and ADC channels to operate. Writing a 1 activates the oscillator. Writing a 0 disables the oscillator. */
#define TCS3472_ENABLE_PON_MSK                  (0b1U << TCS3472_ENABLE_PON_POS)
#define TCS3472_ENABLE_AEN_POS                  0x1U  /** RGBC enable. This bit actives the two-channel ADC. Writing a 1 activates the RGBC. Writing a 0 disables the RGBC. */
#define TCS3472_ENABLE_AEN_MSK                  (0b1U << TCS3472_ENABLE_AEN_POS)
#define TCS3472_ENABLE_WEN_POS                  0x3U  /** Wait enable. This bit activates the wait feature. Writing a 1 activates the wait timer. Writing a 0 disables the wait timer. */
#define TCS3472_ENABLE_WEN_MSK                  (0b1U << TCS3472_ENABLE_WEN_POS)
#define TCS3472_ENABLE_AIEN_POS                 0x4U  /** RGBC interrupt enable. When asserted, permits RGBC interrupts to be generated. */
#define TCS3472_ENABLE_AIEN_MSK                 (0b1U << TCS3472_ENABLE_AIEN_POS)

/** RGBC Timing Register */
#define TCS3472_ATIME_REG                       0x01U
#define TCS3472_ATIME_ATIME_POS                 0x0U
#define TCS3472_ATIME_ATIME_MSK                 (0b11111111U << TCS3472_ATIME_ATIME_POS)

#define TCS3472_INTEGRATIONTIME_2_4MS           (0xFFU << TCS3472_ATIME_ATIME_POS)  /** 2.4ms - 1 cycle    - Max Count: 1024  */
#define TCS3472_INTEGRATIONTIME_24MS            (0xF6U << TCS3472_ATIME_ATIME_POS)  /** 24ms  - 10 cycles  - Max Count: 10240 */
#define TCS3472_INTEGRATIONTIME_50MS            (0xEBU << TCS3472_ATIME_ATIME_POS)  /** 50ms  - 20 cycles  - Max Count: 20480 */
#define TCS3472_INTEGRATIONTIME_101MS           (0xD5U << TCS3472_ATIME_ATIME_POS)  /** 101ms - 42 cycles  - Max Count: 43008 */
#define TCS3472_INTEGRATIONTIME_154MS           (0xC0U << TCS3472_ATIME_ATIME_POS)  /** 154ms - 64 cycles  - Max Count: 65535 */
#define TCS3472_INTEGRATIONTIME_700MS           (0x00U << TCS3472_ATIME_ATIME_POS)  /** 700ms - 256 cycles - Max Count: 65535 */

/** Wait Time Register */
#define TCS3472_WTIME_REG                       0x03U
#define TCS3472_WTIME_WTIME_POS                 0x0U
#define TCS3472_WTIME_WTIME_MSK                 (0b11111111U << TCS3472_WTIME_WTIME_POS)

#define TCS3472_WAIT_1                          0xFFU  /** WLONG = 0 --- 2.4ms; WLONG = 1 --- 29ms*/
#define TCS3472_WAIT_85                         0xABU  /** WLONG = 0 --- 204ms; WLONG = 1 --- 2450ms*/
#define TCS3472_WAIT_256                        0x00U  /** WLONG = 0 --- 614ms; WLONG = 1 --- 7400ms*/

/* RGBC Interrupt Threshold Register */
#define TCS3472_AILTL_REG                       0x04U
#define TCS3472_AILTL_AILTL_POS                 0x0U
#define TCS3472_AILTL_AILTL_MSK                 (0b11111111U << TCS3472_AILTL_AILTL_POS)

#define TCS3472_AILTH_REG                       0x05U
#define TCS3472_AILTH_AILTH_POS                 0x0U
#define TCS3472_AILTH_AILTH_MSK                 (0b11111111U << TCS3472_AILTH_AILTH_POS)

#define TCS3472_AIHTL_REG                       0x06U
#define TCS3472_AIHTL_AIHTL_POS                 0x0U
#define TCS3472_AIHTL_AIHTL_MSK                 (0b11111111U << TCS3472_AIHTL_AIHTL_POS)

#define TCS3472_AIHTH_REG                       0x07U
#define TCS3472_AIHTH_AIHTH_POS                 0x0U
#define TCS3472_AIHTH_AIHTH_MSK                 (0b11111111U << TCS3472_AIHTH_AIHTH_POS)

/** Persistence Register */
#define TCS3472_PERS_REG                        0x0CU
#define TCS3472_PERS_APERS_POS                  0x0U
#define TCS3472_PERS_APERS_MSK                  (0b1111U << TCS3472_PERS_APERS_POS)

#define TCS3472_PERS_NONE                 (0b0000U << TCS3472_PERS_APERS_POS)  ///< Every RGBC cycle generates an interrupt
#define TCS3472_PERS_1_CYCLE              (0b0001U << TCS3472_PERS_APERS_POS)  ///< 1 clean channel value outside threshold range generates an interrupt
#define TCS3472_PERS_2_CYCLE              (0b0010U << TCS3472_PERS_APERS_POS)  ///< 2 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_3_CYCLE              (0b0011U << TCS3472_PERS_APERS_POS)  ///< 3 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_5_CYCLE              (0b0100U << TCS3472_PERS_APERS_POS)  ///< 5 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_10_CYCLE             (0b0101U << TCS3472_PERS_APERS_POS)  ///< 10 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_15_CYCLE             (0b0110U << TCS3472_PERS_APERS_POS)  ///< 15 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_20_CYCLE             (0b0111U << TCS3472_PERS_APERS_POS)  ///< 20 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_25_CYCLE             (0b1000U << TCS3472_PERS_APERS_POS)  ///< 25 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_30_CYCLE             (0b1001U << TCS3472_PERS_APERS_POS)  ///< 30 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_35_CYCLE             (0b1010U << TCS3472_PERS_APERS_POS)  ///< 35 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_40_CYCLE             (0b1011U << TCS3472_PERS_APERS_POS)  ///< 40 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_45_CYCLE             (0b1100U << TCS3472_PERS_APERS_POS)  ///< 45 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_50_CYCLE             (0b1101U << TCS3472_PERS_APERS_POS)  ///< 50 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_55_CYCLE             (0b1110U << TCS3472_PERS_APERS_POS)  ///< 55 clean channel values outside threshold range generates an interrupt
#define TCS3472_PERS_60_CYCLE             (0b1111U << TCS3472_PERS_APERS_POS)  ///< 60 clean channel values outside threshold range generates an interrupt

/** Configuration Register */
#define TCS3472_CONFIG_REG                      0x0DU
#define TCS3472_CONFIG_WLONG_POS                0x1U  /** Wait Long. When asserted, the wait cycles are increased by a factor 12× from that programmed in the WTIME register. */
#define TCS3472_CONFIG_WLONG_MSK                (0b1U << TCS3472_CONFIG_WLONG_POS)

/** Control Register */
#define TCS3472_CONTROL_REG                     0x0FU
#define TCS3472_CONTROL_AGAIN_POS               0x0U  /** RGBC Gain Control. */
#define TCS3472_CONTROL_AGAIN_MSK               (0b11U << TCS3472_CONTROL_AGAIN_POS)

#define TCS3472_GAIN_1X                   (0b00U << TCS3472_CONTROL_AGAIN_POS)
#define TCS3472_GAIN_4X                   (0b01U << TCS3472_CONTROL_AGAIN_POS)
#define TCS3472_GAIN_16X                  (0b10U << TCS3472_CONTROL_AGAIN_POS)
#define TCS3472_GAIN_60X                  (0b11U << TCS3472_CONTROL_AGAIN_POS)

/** ID Register */
#define TCS3472_ID_REG                          0x12U
#define TCS3472_ID_ID_POS                       0x0U  /** Part number identification */
#define TCS3472_ID_ID_MSK                       (0b11111111U << TCS3472_ID_ID_POS)

#define TCS34721_ID_VALUE                       0x44U
#define TCS34725_ID_VALUE                       0x44U
#define TCS34723_ID_VALUE                       0x4DU
#define TCS34727_ID_VALUE                       0x4DU

/** Status Register */
#define TCS3472_STATUS_REG                      0x13UL
#define TCS3472_STATUS_AVALID_POS               0x0U  /** RGBC Valid. Indicates that the RGBC channels have completed an integration cycle. */
#define TCS3472_STATUS_AVALID_MSK               (0b1U << TCS3472_STATUS_AVALID_POS)
#define TCS3472_STATUS_AINT_POS                 0x4U  /** RGBC clear channel Interrupt. */
#define TCS3472_STATUS_AINT_MSK                 (0b1U << TCS3472_STATUS_AINT_POS)

/* RGBC Channel Data Registers */
#define TCS3472_CDATAL_REG                      0x14U
#define TCS3472_CDATAH_REG                      0x15U
#define TCS3472_RDATAL_REG                      0x16U
#define TCS3472_RDATAH_REG                      0x17U
#define TCS3472_GDATAL_REG                      0x18U
#define TCS3472_GDATAH_REG                      0x19U
#define TCS3472_BDATAL_REG                      0x1AU
#define TCS3472_BDATAH_REG                      0x1BU

namespace Rath {

class TCS3472 {
  public:

    /**
     * @brief 
     * 
     * @param mem_address 
     * @param buffer 
     * @param size 
     */
    void readMemory(uint8_t mem_address, uint8_t *buffer, uint16_t size);

    /**
     * @brief 
     * 
     * @param mem_address 
     * @param buffer 
     * @param value 
     */
    void writeMemory(uint8_t mem_address, uint8_t *buffer, uint16_t size);

    /**
     * @brief Initialize the I2C communcation with the TCS3472 module.
     *
     */
    void init();

    /**
     * @brief Enable the TCS3472 from power down mode or sleep mode. By default, TCS3472 is in power down mode on bootup.
     *
     * This function will set the Gain and Integration Time to the default TCS3472_GAIN_1X and TCS3472_INTEGRATIONTIME_2_4MS
     *
     * @return uint8_t
     */
    void enable();

    /**
     * @brief Test if the TCS3472 is connected.
     *
     * @return uint8_t the ID of the device or 0 if not connected
     */
    uint8_t isConnected();

    /**
     * @brief Get the specific device type from the ID.
     *
     * @return char* name of the device
     */
    char* getDeviceName();

    /**
     * @brief Set the integration time.
     *
     * @param value
     */
    void setIntegrationTime(uint8_t value);

    /**
     * @brief Set the gain
     *
     * @param value
     */
    void setGain(uint8_t value);

    /**
     * @brief Get the current state of the onboard LED.
     *
     * @return uint8_t the state of the LED
     */
    uint8_t getLED();

    /**
     * @brief Set the state of the onboard LED.
     *
     * @param state
     */
    void setLED(uint8_t state);

    /**
     * @brief Get the RGBA reading from the sensor.
     *
     * @param buffer a uint16_t array of size 4 to store R, G, B, and Alpha value
     */
    void get(uint16_t *buffer);

    /**
     * @brief Get the RGBA reading from the sensor.
     *
     * @param R
     * @param G
     * @param B
     * @param alpha
     */
    void get(uint16_t *R, uint16_t *G, uint16_t *B, uint16_t *alpha);
    
    void delay(uint32_t time);

  private:
    uint8_t intergration_time = TCS3472_INTEGRATIONTIME_50MS;
    uint8_t gain = TCS3472_GAIN_1X;
    uint32_t timeout = 1000;
};

}
#endif  // __RATH_TCS3472_H
