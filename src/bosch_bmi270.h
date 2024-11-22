/*****************************************************************************/
/**
 * @file       bosch_bmi270.h
 * @brief      bosch bmi270 sensor library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
 * @date       2024-11-15
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include "BMI270_SensorAPI/bmi270.h"
#include "BMI270_SensorAPI/bmi2_defs.h"

#include <Arduino.h>
#include <Wire.h>

#include <cstdint>

/*****************************************************************************/
namespace andrgrue::sensor {

/*****************************************************************************/

/**
 * @brief bosch bmi270 sensor driver for Arduino Nano uController
 */
class bosch_bmi270 {
  // data types
public:
  /**
   * @brief sensor data
   */
  struct Data {
    float    x;
    float    y;
    float    z;
    float    variance;
    uint64_t timestamp;
    uint8_t  flags;
  };

  /// device information
  struct DevInfo {
    TwoWire* wire;
    uint8_t  dev_addr;
  };

  // construction
public:
  /**
   * @brief constructor
   * @param wire I2C bus
   * @param address I2C device address of sensor
   * @param irqPin interrupt pin name
   * @param accelerationVariance variance in (m/s^2)^2
   * @param angularVariance variance in (deg/s)^2
   */
  bosch_bmi270(TwoWire&       wire,
               const uint8_t& address              = BMI2_I2C_PRIM_ADDR,
               const PinName& irqPin               = NC,
               const float    accelerationVariance = 0.00025,
               const float    angularrateVariance  = 0.0064);
  virtual ~bosch_bmi270() = default;

  // operations
public:
  /**
   * @brief initialize sensor
   * @param cb interrupt callback
   */
  bool initialize(const uint8_t& acc_freq   = BMI2_ACC_ODR_25HZ,
                  const uint8_t& acc_range  = BMI2_ACC_RANGE_4G,
                  const uint8_t& gyro_freq  = BMI2_GYR_ODR_25HZ,
                  const uint8_t& gyro_range = BMI2_GYR_RANGE_2000
#ifdef __MBED__
                  ,
                  mbed::Callback<void(void)> cb = nullptr
#endif
  );
  void terminate();

  /**
   * @brief enable interrupt generation
   * interrupt pin must be configured
   */
  void enableInterrupt();

  /**
   * @brief disable interrupt generation
   */
  void disableInterrupt();

#if __MBED__
  /**
   * @brief link interrupt callback
   * @param cb interrupt callback
   * @return true if successful
   */
  bool interruptCallback(mbed::Callback<void()> cb);
#endif

  void setContinuousMode();
  void oneShotMode();

  /**
   * @brief check if new data is available
   * @return true if new data is available
   */
  bool accelerationAvailable();
  /**
   * @brief read acceleration
   * @param x x-axis acceleration in m/s^2
   * @param y y-axis acceleration in m/s^2
   * @param z z-axis acceleration in m/s^2
   */
  int acceleration(float& x, float& y, float& z);
  int acceleration(Data& data);
  /// Sampling rate of the sensor.
  float accelerationSampleRate();

  /**
   * @brief check if new data is available
   * @return true if new data is available
   */
  bool angularrateAvailable();
  /**
   * @brief read angular rate
   * @param x x-axis angular rate in degrees/second
   * @param y y-axis angular rate in degrees/second
   * @param z z-axis angular rate in degrees/second
   */
  int angularrate(float& x, float& y, float& z);
  int angularrate(Data& data);
  /// Sampling rate of the sensor.
  float angularrateSampleRate();

protected:
  int8_t configure_sensor(struct bmi2_dev* dev);
#if __MBED__
  void interruptHandler();
#endif

private:
  // data
public:
protected:
private:
  TwoWire*      wire_;
  const uint8_t address_ {0};
  const PinName irqPin_ {NC};
  bool          initialized_ {false};
  bool          continuousMode_ {false};
#ifdef __MBED__
  mbed::Callback<void(void)> cb_;
#endif
  volatile uint16_t int_status_ {0};
  volatile uint64_t timestamp_ns_ {0};

  DevInfo         accel_gyro_dev_info;
  struct bmi2_dev bmi2;

  uint8_t acc_freq_ {BMI2_ACC_ODR_100HZ};
  uint8_t acc_range_ {BMI2_ACC_RANGE_4G};
  uint8_t gyro_freq_ {BMI2_GYR_ODR_100HZ};
  uint8_t gyro_range_ {BMI2_GYR_RANGE_2000};

  /**
   * @brief variance of the sensor values
   *  expected bandwith 100Hz
   *  noise density = 160ug/sqrt(Hz) and 0.008 dps/sqrt(Hz)
   *  --> stdev = 0.0016g (0.0157m/s^2) and 0.08dps
   *  --> variance = 0.00025m^2/s^4 and 0.0064dps^2
   */
  const float accelerationVariance_ {0.0f};
  const float angularrateVariance_ {0.0f};
};

/*****************************************************************************/
}  // namespace andrgrue::sensor
