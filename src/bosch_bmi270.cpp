/*****************************************************************************/
/**
 * @file       bosch_bmi270.cpp
 * @brief      bosch bmi270 sensor library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
 * @date       2024-11-15
 */
/*****************************************************************************/
#include "bosch_bmi270.h"

#include "Arduino.h"
#include "Wire.h"

#ifdef __MBED__
#  include "drivers/InterruptIn.h"
#  include "mbed_events.h"
#  include "mbed_shared_queues.h"

static events::EventQueue accel_gyro_queue(10 * EVENTS_EVENT_SIZE);
#endif

/*****************************************************************************/
namespace andrgrue::sensor {

/*****************************************************************************/

static int8_t bmi2_i2c_read(uint8_t  reg_addr,
                            uint8_t* reg_data,
                            uint32_t len,
                            void*    intf_ptr) {
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  bosch_bmi270::DevInfo* dev_info = (bosch_bmi270::DevInfo*)intf_ptr;
  uint8_t                dev_id   = dev_info->dev_addr;

  dev_info->wire->beginTransmission(dev_id);
  dev_info->wire->write(reg_addr);
  if (dev_info->wire->endTransmission(false) == 0) {
    bytes_received = dev_info->wire->requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++) {
      reg_data[i] = dev_info->wire->read();
    }
  } else {
    return -1;
  }

  return 0;
}

static int8_t bmi2_i2c_write(uint8_t        reg_addr,
                             const uint8_t* reg_data,
                             uint32_t       len,
                             void*          intf_ptr) {
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  bosch_bmi270::DevInfo* dev_info = (bosch_bmi270::DevInfo*)intf_ptr;
  uint8_t                dev_id   = dev_info->dev_addr;
  dev_info->wire->beginTransmission(dev_id);
  dev_info->wire->write(reg_addr);
  for (uint16_t i = 0; i < len; i++) {
    dev_info->wire->write(reg_data[i]);
  }
  if (dev_info->wire->endTransmission() != 0) {
    return -1;
  }

  return 0;
}

static void bmi2_delay_us(uint32_t period, void* intf_ptr) {
  delayMicroseconds(period);
}

/*****************************************************************************/

bosch_bmi270::bosch_bmi270(TwoWire&       wire,
                           const uint8_t& address,
                           const PinName& irqPin,
                           const float    accelerationVariance,
                           const float    angularrateVariance)
    : wire_(&wire)
    , address_(address)
    , irqPin_(irqPin)
    , accelerationVariance_(accelerationVariance)
    , angularrateVariance_(angularrateVariance) {}

bool bosch_bmi270::initialize(const uint8_t& acc_freq,
                              const uint8_t& acc_range,
                              const uint8_t& gyro_freq,
                              const uint8_t& gyro_range
#ifdef __MBED__
                              ,
                              mbed::Callback<void(void)> cb
#endif
) {
  acc_freq_   = acc_freq;
  acc_range_  = acc_range;
  gyro_freq_  = gyro_freq;
  gyro_range_ = gyro_range;

#ifdef __MBED__
  if (!interruptCallback(cb)) {
    initialized_ = false;
    return false;
  }
#endif
  enableInterrupt();

  bmi2.chip_id         = address_;
  bmi2.read            = bmi2_i2c_read;
  bmi2.write           = bmi2_i2c_write;
  bmi2.delay_us        = bmi2_delay_us;
  bmi2.intf            = BMI2_I2C_INTF;
  bmi2.intf_ptr        = &accel_gyro_dev_info;
  bmi2.read_write_len  = 30;    // Limitation of the Wire library
  bmi2.config_file_ptr = NULL;  // Use the default BMI270 config file

  accel_gyro_dev_info.wire     = wire_;
  accel_gyro_dev_info.dev_addr = address_;

  int8_t bmi270InitResult = bmi270_init(&bmi2);
  if (bmi270InitResult != BMI2_OK)
    return false;

  int8_t bmi270ConfigResult = configure_sensor(&bmi2);
  if (bmi270ConfigResult != BMI2_OK)
    return false;

  bool success =
      (bmi270InitResult == BMI2_OK) && (bmi270ConfigResult == BMI2_OK);
  initialized_ = success;
  return success;
}

void bosch_bmi270::terminate() { initialized_ = false; }

int8_t bosch_bmi270::configure_sensor(struct bmi2_dev* dev) {
  int8_t  rslt;
  uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type             = BMI2_INT1;
  int_pin_cfg.int_latch            = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od        = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type                = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp         = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr         = acc_freq_;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[0].cfg.acc.range       = acc_range_;
  sens_cfg[1].type                = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp         = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr         = gyro_freq_;
  sens_cfg[1].cfg.gyr.range       = gyro_range_;
  sens_cfg[1].cfg.gyr.ois_range   = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}

void bosch_bmi270::enableInterrupt() {
  // configured as an input
  if (NC != irqPin_) {
    pinMode(irqPin_, INPUT);
  }
}

void bosch_bmi270::disableInterrupt() {}

#ifdef __MBED__

bool bosch_bmi270::interruptCallback(mbed::Callback<void(void)> cb) {
  if (NC != irqPin_ && nullptr != cb) {
    static mbed::InterruptIn irq(irqPin_, PullDown);
    static rtos::Thread      event_t(osPriorityHigh, 768, nullptr, "events");
    cb_          = cb;
    osStatus ret = event_t.start(
        callback(&accel_gyro_queue, &events::EventQueue::dispatch_forever));
    if (osOK == ret) {
      irq.rise(mbed::callback(this, &bosch_bmi270::interruptHandler));
      return true;
    }
    return false;
  }
  return true;
}

void bosch_bmi270::interruptHandler() {
  if (initialized_ && cb_) {
    timestamp_ns_ = micros() * 1000ULL;
    accel_gyro_queue.call(cb_);
  }
}

#endif

void bosch_bmi270::setContinuousMode() {
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 1, &bmi2);
  continuousMode_ = true;
}

void bosch_bmi270::oneShotMode() {
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 0, &bmi2);
  continuousMode_ = false;
}

int bosch_bmi270::acceleration(float& x, float& y, float& z) {
  constexpr float GRAVITY = 9.8067;
  // default range is +-4G, so conversion factor is (((1 << 15)/4.0f))
  const float factor = ((1 << 15) / (1 << (1 + acc_range_))) / GRAVITY;
  struct bmi2_sens_data sensor_data;
  auto                  ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  x                         = sensor_data.acc.x / factor;
  y                         = sensor_data.acc.y / factor;
  z                         = sensor_data.acc.z / factor;
  return (ret == 0);
}

int bosch_bmi270::acceleration(Data& data) {
  data.variance  = accelerationVariance_;
  data.timestamp = timestamp_ns_;
  data.flags     = 0;
  return acceleration(data.x, data.y, data.z);
}

bool bosch_bmi270::accelerationAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret     = ((status | int_status_) & BMI2_ACC_DRDY_INT_MASK);
  int_status_ = status;
  int_status_ &= ~BMI2_ACC_DRDY_INT_MASK;
  return ret != 0;
}

float bosch_bmi270::accelerationSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_ACCEL;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}

int bosch_bmi270::angularrate(float& x, float& y, float& z) {
  // default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
  const float           factor = ((1 << 15) / 2000.0f) * (1 << gyro_range_);
  struct bmi2_sens_data sensor_data;
  auto                  ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  x                         = sensor_data.gyr.x / factor;
  y                         = sensor_data.gyr.y / factor;
  z                         = sensor_data.gyr.z / factor;
  return (ret == 0);
}

int bosch_bmi270::angularrate(Data& data) {
  data.variance  = angularrateVariance_;
  data.timestamp = timestamp_ns_;
  data.flags     = 0;
  return angularrate(data.x, data.y, data.z);
}

bool bosch_bmi270::angularrateAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret     = ((status | int_status_) & BMI2_GYR_DRDY_INT_MASK);
  int_status_ = status;
  int_status_ &= ~BMI2_GYR_DRDY_INT_MASK;
  return ret != 0;
}

float bosch_bmi270::angularrateSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_GYRO;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}

/*****************************************************************************/
}  // namespace andrgrue::sensor
