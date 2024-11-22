`Arduino_BOSCH_BMI270`
====================

This is an Arduino library for the [BOSCH BMI270](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270) IMU sensor
like it is used in the [Arduino Nano 33 BLE Sense](https://www.st.com/en/development-tools/nano-33-ble-sense.html).
The library supports several ways of reading the sensor data,
including the provided mechanisms of the sensor like One-Shot mode, Continuous mode and also Interrupt generation.
A main goal of the library is to provide a simple and also efficent way of reading the sensor data.


This library uses the [BOSCH BMI270 Sensor API]( https://github.com/boschsensortec/BMI270_SensorAPI).

## Data sheets
- [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)

## Usage

### Object creation

```cpp
#include "bosch_bmi270.h"
andrgrue::sensor::bosch_bmi270 Imu(Wire1,
                                   BMI2_I2C_PRIM_ADDR,
                                   p11);

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
andrgrue::sensor::bosch_bmi270::Data acc_data;
andrgrue::sensor::bosch_bmi270::Data gyro_data;
volatile bool imuInterruptFlag = false;
```

### Interrupt handler

```c++
void imu_interrupt_handler() {
  imuInterruptFlag = true;
}
```

### Setup

```c++
void setup() {

  // wire must be initialized first
  Wire1.begin();
  Wire1.setClock(400000);

  imuInterruptFlag = false;
  if(!Imu.initialize(BMI2_ACC_ODR_100HZ, BMI2_ACC_RANGE_8G,
                     BMI2_GYR_ODR_100HZ, BMI2_GYR_RANGE_2000,
                     BMM150_DATA_RATE_25HZ,
                     imu_interrupt_handler)) {
    Serial.println("IMU init failed!");
    while (1);
  }

  Serial.print("sample rate: acc=");
  Serial.println(Imu.accelerationSampleRate());
  Serial.print("sample rate: gyro=");
  Serial.println(Imu.angularrateSampleRate());
}
```

### Loop

```c++
void loop() {

  if(imuInterruptFlag){
    if(Imu.accelerationAvailable()){
      Imu.acceleration(acc_data);
      acc_count++;
    }
    if(Imu.angularrateAvailable()){
      Imu.angularrate(gyro_data);
      gyro_count++;
    }
    imuInterruptFlag = false;
  }

}
```

## Credits

This project was inspired and includes several elements from the following projects.
Special thanks to the authors.

 - [Arduino_BMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150)

## License

Copyright © 2024, André Grüttner. All rights reserved.

This project is licensed under the GNU Lesser General Public License v2.1 (LGPL-2.1).
You may use, modify, and distribute this software under the terms of the LGPL-2.1 license.
See the [LICENSE](./LICENSE.txt) file for details, or visit [GNU’s official LGPL-2.1 page](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html) for the full license text.
