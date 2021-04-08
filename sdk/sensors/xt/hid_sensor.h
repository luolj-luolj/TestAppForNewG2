/*
 * Copyright 2019 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CARDBOARD_SDK_SENSORS_XT_HID_SENSOR_H_
#define CARDBOARD_SDK_SENSORS_XT_HID_SENSOR_H_

#include <mutex>
#include "libs/include/libusb.h"
#include "util/vector.h"

namespace cardboard {
#ifndef G
#define G                              9.81188f
#endif

#ifndef PI
#define PI                             3.14159f
#endif

/**
 * Sensor Temprature resolution in deg Celsius/LSB
 * 1/326.8 deg Celsius per LSB
 */
#define ICM206XX_SENSOR_TEMPERATURE_RESOLUTION  (0.00306f)
#define ICM206XX_ROOM_TEMP_OFFSET                       25//Celsius degree


/**
 * Accelerometer resolutions (mg/LSB)
 */
#define ICM206XX_ACCEL_RESOLUTION_2G    (0.0610f)
#define ICM206XX_ACCEL_RESOLUTION_4G    (0.1221f)
#define ICM206XX_ACCEL_RESOLUTION_8G    (0.2441f)
#define ICM206XX_ACCEL_RESOLUTION_16G   (0.4883f)

/**
 * ICM206XX sensitivity for gyro in dps/LSB
 */
#define ICM206XX_GYRO_SSTVT_250DPS              (0.00763)
#define ICM206XX_GYRO_SSTVT_500DPS              (0.01526)
#define ICM206XX_GYRO_SSTVT_1000DPS             (0.03052)
#define ICM206XX_GYRO_SSTVT_2000DPS             (0.06104)

#define CALI_FILE_LEN 4096
#define FILE_NAME "/sdcard/DCIM/hmdinfo.tar.gz"
static constexpr uint64_t kNanosInSeconds = 1000000000;
constexpr uint64_t kPredictionTimeWithoutVsyncNanos = 50000000;

#define ST_LOG_DEBUG 0

class HidSensor {
 public:
  static HidSensor *&GetInstance();

  static void DeleteInstance();

  int setParams(int vid, int pid, int fd, int busnum, int devaddr, const char *usbfs);
  bool init();
  bool open();
  void close();
  void exit();

  int startReading(unsigned char *buffer, int length, unsigned int timeout);

  int startReading2(unsigned char *buffer, int length, unsigned int timeout);

  int startReadingCaliData(int type, std::array<float, 3>& cali_data);

  int setStUfd(int vid, int pid, int fd, int busnum, int devaddr, const char* usbfs_str);
  int initSt();
  int openSt();
  void closeSt(void);
  void exitSt();
  int handShakeWithSt(void);
  int sendCommandToSt(unsigned char command, unsigned char value);
  int sendDataToSt(int x, int y);
  int receiveDataFromStControlChannel(void);
  int receiveDataFromStDataChannel(void);

  uint64_t getTimeNano();
  int convertAxis(const int axis[3][3], float out[3], float in[3]);

  private:
  HidSensor();
  ~HidSensor();

  HidSensor(const HidSensor &signal);

  const HidSensor &operator=(const HidSensor &signal);

private:
  static HidSensor *m_SingleInstance;
  static std::mutex m_Mutex;
};
}
#endif //CARDBOARD_SDK_SENSORS_XT_HID_SENSOR_H_
