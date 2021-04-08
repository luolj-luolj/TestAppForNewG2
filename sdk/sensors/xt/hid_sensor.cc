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
#include "hid_sensor.h"

#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/system_properties.h>
#include "Calibration.h"

#include "util/logging.h"

Vector3f AccelOffset, GyroOffset;
float AccBias[3];
float GyroBias[3];
float R[3][3];
Calibration calibration;
// Factory calibration data
Matrix3f AccelMatrix, GyroMatrix;
static const int b_axis[3][3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}; //before calibration
static const int a_axis[3][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}}; //after calibration
namespace cardboard {
HidSensor *HidSensor::m_SingleInstance = nullptr;
std::mutex HidSensor::m_Mutex;

static int mOvVid, mOvPid, mOvFd, mOvBusnum, mOvDevaddr;
static char *mOvUsbfs = nullptr;
static bool ov_opened;

static struct libusb_context *ov_usb_ctx = nullptr;
static struct libusb_device *ov_usb_dev;
static struct libusb_device_handle *ov_usb_handle;
static struct libusb_config_descriptor *ov_conf_desc;
static int ov_done = 0;
static int interface_num;

static struct libusb_context *st_usb_ctx = nullptr;
static struct libusb_device *st_usb_dev;
static struct libusb_device_handle *st_usb_handle;
static struct libusb_config_descriptor *st_conf_desc;

static int kStInf = 4;
static char kCaliBuf[CALI_FILE_LEN];
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t file_write_t;
static void* caliFile_write(void *arg);

static int mStVid, mStPid, mStFd, mStBusnum, mStDevaddr;
static char *mStUsbfs = nullptr;
static int st_output_endpoint;
static int st_input_endpoint;
static int st_inf_num;
static bool st_opened = false;
static int st_done = 0;

static pthread_t ov_hotplug_t;
static pthread_attr_t ov_attr;
static void* ov_hotplug_monitor_thread(void *arg);
static int ov_monitor_running = 0;

static pthread_t st_hotplug_t;
static pthread_attr_t st_attr;
static void* st_hotplug_monitor_thread(void *arg);
static int st_monitor_running = 0;

void internel_close(int who);
void internel_exit(int who);

HidSensor *&HidSensor::GetInstance()
{
    if (m_SingleInstance == nullptr)
     {
        std::unique_lock<std::mutex> lock(m_Mutex);
        if (m_SingleInstance == nullptr)
        {
            m_SingleInstance = new (std::nothrow) HidSensor;
        }
     }
   return m_SingleInstance;
}

void HidSensor::DeleteInstance()
{
    std::unique_lock<std::mutex> lock(m_Mutex);
    if (m_SingleInstance)
    {
        delete m_SingleInstance;
        m_SingleInstance = nullptr;
    }
}

HidSensor::HidSensor()
{
}

HidSensor::~HidSensor(){
    if (ov_monitor_running) {
        pthread_attr_destroy(&ov_attr);
        pthread_kill(ov_hotplug_t, 0);
    }

    if (st_monitor_running) {
        pthread_attr_destroy(&st_attr);
        pthread_kill(st_hotplug_t, 0);
    }

    internel_close(0);
    internel_exit(0);
    internel_close(1);
    internel_exit(1);
}

void internel_close(int who) {
    int res;
    if (who == 0) {
        if (!ov_opened) {
            PHONEAR_LOGI("ov libusb_already closed");
            return;
        }
        if (ov_opened) {
            if (interface_num > 0) {
                /* Release the interface */
                res = libusb_release_interface(ov_usb_handle, interface_num);
                if (res < 0)
                    PHONEAR_LOGE("Can't release the interface.\n");
                interface_num = -1;
            }
            if (ov_conf_desc != nullptr) {
                libusb_free_config_descriptor(ov_conf_desc);
                ov_conf_desc = nullptr;
            }
            if (ov_usb_handle != nullptr) {
                libusb_close(ov_usb_handle);
                ov_usb_handle = nullptr;
            }
            ov_opened = false;
            PHONEAR_LOGI("close ov successfully");
        }
    } else if (who == 1) {
        if (st_opened) {
            if (st_inf_num > 0) {
                /* Release the interface */
                res = libusb_release_interface(st_usb_handle, st_inf_num);
                if (res < 0)
                    PHONEAR_LOGE("Can't release the st interface.\n");
                st_inf_num = -1;
                st_output_endpoint = -1;
                st_input_endpoint = -1;
            }
            if (st_conf_desc != nullptr) {
                libusb_free_config_descriptor(st_conf_desc);
                st_conf_desc = nullptr;
            }
            if (st_usb_handle != nullptr) {
                libusb_close(st_usb_handle);
                st_usb_handle = nullptr;
            }
            st_opened = false;
            PHONEAR_LOGI("close st successfully");
        } else {
            PHONEAR_LOGE("st already closed\n");
        }
    } else {
        PHONEAR_LOGE("close who is incorrect\n");
    }
}

void internel_exit(int who) {
    if (who == 0) {
        if (mOvUsbfs != nullptr) {
            free(mOvUsbfs);
            mOvUsbfs = nullptr;
        }
        if(ov_usb_ctx != nullptr) {
            libusb_exit(ov_usb_ctx);
            ov_usb_ctx = nullptr;
        }
    } else if (who == 1) {
        if (mStUsbfs != nullptr) {
            free(mStUsbfs);
            mStUsbfs = nullptr;
        }
        if(st_usb_ctx != nullptr) {
            libusb_exit(st_usb_ctx);
            st_usb_ctx = nullptr;
        }
    } else {
        PHONEAR_LOGE("exit who is incorrect\n");
    }
}

int LIBUSB_CALL hotplug_callback(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data)
{
    struct libusb_device_descriptor desc;
    int rc;

    (void) ctx;
    (void) dev;
    (void) event;
    (void) user_data;

    rc = libusb_get_device_descriptor(dev, &desc);
    if (LIBUSB_SUCCESS != rc) {
        PHONEAR_LOGE("Error getting device descriptor");
    }

    PHONEAR_LOGI("Device attached: %04x:%04x\n", desc.idVendor, desc.idProduct);

    if ((desc.idVendor == 0x05A9) || (desc.idProduct == 0x0F87)) {
        PHONEAR_LOGI("wait ov operation from service");
        ov_done++;
    }

    if ((desc.idVendor == 0x0483) || (desc.idProduct == 0x7705)) {
        PHONEAR_LOGI("wait st operation from service");
        st_done++;
    }

    return 0;
}

int LIBUSB_CALL hotplug_callback_detach(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data)
{
    struct libusb_device_descriptor desc;
    int rc;

    (void) ctx;
    (void) dev;
    (void) event;
    (void) user_data;

    rc = libusb_get_device_descriptor(dev, &desc);
    if (LIBUSB_SUCCESS != rc) {
        PHONEAR_LOGE("Error getting device descriptor");
    }

    PHONEAR_LOGI("Device detached: %04x:%04x\n", desc.idVendor, desc.idProduct);

    if ((desc.idVendor == 0x05A9) || (desc.idProduct == 0x0F87)) {
        internel_close(0);
        internel_exit(0);
        ov_done++;
    }

    if ((desc.idVendor == 0x0483) || (desc.idProduct == 0x7705)) {
        internel_close(1);
        internel_exit(1);
        st_done++;
    }
    return 0;
}

int HidSensor::setParams(int vid, int pid, int fd, int busnum, int devaddr, const char *usbfs) {
    if (ov_opened) {
        //check whether usbfs is the same
        if ((mOvUsbfs != nullptr) && (strcmp(mOvUsbfs, usbfs) == 0) && (mOvFd == fd)
            && (mOvBusnum == busnum) && (mOvDevaddr == devaddr)) {
            PHONEAR_LOGE("setParams repeat, ignore current = %s, %d, %d, %d, new = %s, %d, %d, %d",
                         mOvUsbfs, mOvFd, mOvBusnum, mOvDevaddr, usbfs, fd, busnum, devaddr);
            return LIBUSB_ERROR_BUSY;
        } else {
            // close first
            PHONEAR_LOGE("setParams close previous device first");
            close();
        }
    }
    mOvVid = vid;
    mOvPid = pid;
    mOvFd = fd;
    mOvBusnum = busnum;
    mOvDevaddr = devaddr;
    if (mOvUsbfs != nullptr) {
        free(mOvUsbfs);
    }
    mOvUsbfs = strdup(usbfs);
    PHONEAR_LOGI("setParams: new ov usbfs = %s, ovFd = %d", mOvUsbfs, mOvFd);
    return LIBUSB_SUCCESS;
}

void* ov_hotplug_monitor_thread(void *arg) {
    // register hotplug callback
    libusb_hotplug_callback_handle hp[2];
    ov_monitor_running = 1;
    PHONEAR_LOGI("ov_hotplug_monitor_thread running");
    int rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
                                              LIBUSB_HOTPLUG_NO_FLAGS, mOvVid,
                                              mOvPid, LIBUSB_HOTPLUG_MATCH_ANY,
                                              hotplug_callback, NULL, &hp[0]);
    if (LIBUSB_SUCCESS != rc) {
        PHONEAR_LOGE("ov Error registering callback 0");
    }

    rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                                          LIBUSB_HOTPLUG_NO_FLAGS, mOvVid,
                                          mOvPid, LIBUSB_HOTPLUG_MATCH_ANY,
                                          hotplug_callback_detach, NULL, &hp[1]);
    if (LIBUSB_SUCCESS != rc) {
        PHONEAR_LOGE("ov Error registering callback 1");
    }

    while (ov_done < 2) {
        rc = libusb_handle_events (NULL);
        if (rc < 0)
            PHONEAR_LOGE("ov libusb_handle_events() failed: %s");
    }
    PHONEAR_LOGI("ov_hotplug_monitor_thread exit");
    ov_monitor_running = 0;
    return NULL;
}

void* st_hotplug_monitor_thread(void *arg) {
    libusb_hotplug_callback_handle hp[2];
    st_monitor_running = 1;
    PHONEAR_LOGI("st_hotplug_monitor_thread running");
    int rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
                                              LIBUSB_HOTPLUG_NO_FLAGS, mStVid,
                                              mStPid, LIBUSB_HOTPLUG_MATCH_ANY,
                                              hotplug_callback, NULL, &hp[0]);
    if (LIBUSB_SUCCESS != rc) {
        PHONEAR_LOGE("st Error registering callback 0");
    }

    rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                                          LIBUSB_HOTPLUG_NO_FLAGS, mStVid,
                                          mStPid, LIBUSB_HOTPLUG_MATCH_ANY,
                                          hotplug_callback_detach, NULL, &hp[1]);
    if (LIBUSB_SUCCESS != rc) {
        PHONEAR_LOGE("st Error registering callback 1");
    }

    while (st_done < 2) {
        rc = libusb_handle_events(NULL);
        if (rc < 0)
            PHONEAR_LOGE("st libusb_handle_events() failed: %s");
    }
    PHONEAR_LOGI("st_hotplug_monitor_thread exit");
    st_monitor_running = 0;
    return NULL;
}

bool HidSensor::init() {
    int res;
    if (mOvUsbfs && strlen(mOvUsbfs) > 0) {
        if (ov_usb_ctx != nullptr) {
            PHONEAR_LOGE("init again, exit previous ov_usb_ctx = %p", ov_usb_ctx);
            libusb_exit(ov_usb_ctx);
            ov_usb_ctx = nullptr;
        }
        res = libusb_init2(&ov_usb_ctx, mOvUsbfs);
        if (res == LIBUSB_SUCCESS) {
            PHONEAR_LOGI("libusb_init success ov_usb_ctx = %p", ov_usb_ctx);
            int ret = libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG);
            if (!ret) {
                PHONEAR_LOGE("Ov Hotplug capabilites are not supported on this platform");
            } else {
                PHONEAR_LOGI("ov libusb_has_capability = %d", ret);
                if (ov_monitor_running == 0) {
                    pthread_attr_init (&ov_attr);
                    pthread_attr_setdetachstate(&ov_attr, PTHREAD_CREATE_DETACHED);
                    pthread_create(&ov_hotplug_t, &ov_attr, ov_hotplug_monitor_thread, NULL);
                }
            }
            PHONEAR_LOGI("ov libusb_init return");
            return true;
        } else {
            PHONEAR_LOGE("libusb_init failed");
            return false;
        }
    } else {
        PHONEAR_LOGE("usbfs incorrect");
        return false;
    }
}

bool HidSensor::open() {
    std::unique_lock<std::mutex> lock(m_Mutex);
    PHONEAR_LOGI("HidSensor opened ? %d", ov_opened);
    if (ov_opened) {
        PHONEAR_LOGI("libusb_already opened");
        return true;
    } else {
        int i = 0;
        char serialNumber[128] = "00000000";
        char accfilePath[150] = "/sdcard/DCIM/AccelBias.txt";
        char gyrofilePath[150] = "/sdcard/DCIM/GyroBias.txt";
        __system_property_get("ro.serialno", serialNumber);
        for (i = 0; i < 3; i++) {
            AccBias[i] = 0.0f;
            GyroBias[i] = 0.0f;
        }
        R[0][0] = R[1][1] = R[2][2] = 1.0f;
        R[0][1] = R[1][0] = R[2][0] = 0.0f;
        R[0][2] = R[1][2] = R[2][1] = 0.0f;
        calibration.Initialize(serialNumber, accfilePath, gyrofilePath);
        calibration.LoadConfigFile(AccBias, GyroBias, R);
        AccelOffset = Vector3f(AccBias[0], AccBias[1], AccBias[2]);
        GyroOffset =  Vector3f(GyroBias[0], GyroBias[1], GyroBias[2]);
        PHONEAR_LOGI("AccelBias[%f,%f,%f]\nGyroBias[%f,%f,%f]", AccBias[0], AccBias[1],
                     AccBias[2], GyroBias[0], GyroBias[1], GyroBias[2]);
    }
    ov_usb_dev = libusb_get_device_with_fd(ov_usb_ctx, mOvVid, mOvPid, nullptr, mOvFd, mOvBusnum, mOvDevaddr);
    if ((!ov_opened) && (ov_usb_dev != nullptr)) {
        struct libusb_device_descriptor desc;
        int res;
        res = libusb_open(ov_usb_dev, &ov_usb_handle);
        if (res == LIBUSB_SUCCESS) {
            PHONEAR_LOGI("libusb_open success");
            libusb_get_device_descriptor(ov_usb_dev, &desc);
            res = libusb_get_active_config_descriptor(ov_usb_dev, &ov_conf_desc);
            if (res < 0)
                res = libusb_get_config_descriptor(ov_usb_dev, 0, &ov_conf_desc);
            if (res == LIBUSB_SUCCESS) {
                PHONEAR_LOGI("libusb_get_config_descriptor success");
                if (ov_conf_desc) {
                    for (int j = 0; j < ov_conf_desc->bNumInterfaces; j++) {
                        const struct libusb_interface *intf = &ov_conf_desc->interface[j];
                        for (int k = 0; k < intf->num_altsetting; k++) {
                            const struct libusb_interface_descriptor *intf_desc;
                            intf_desc = &intf->altsetting[k];
                            if (intf_desc->bInterfaceClass == LIBUSB_CLASS_HID) {
                                interface_num = intf_desc->bInterfaceNumber;
                                PHONEAR_LOGI("IMU sensor found, interface_num is %d",
                                             interface_num);
                                unsigned char data[256];
                                int detached = 0;
                                res = libusb_set_auto_detach_kernel_driver(ov_usb_handle, 1);
                                PHONEAR_LOGI("IMU libusb_set_auto_detach_kernel_driver %d",
                                             res);
                                res = libusb_claim_interface(ov_usb_handle, interface_num);
                                if (res >= 0) {
                                    ov_opened = true;
                                } else {
                                    PHONEAR_LOGE("libusb_claim_interface failed");
                                }
                            } /* find class HID */
                        }
                    }
                }
            } else {
                PHONEAR_LOGE("libusb_get_config_descriptor failed");
                libusb_free_config_descriptor(ov_conf_desc);
            }
        } else {
            PHONEAR_LOGE("ov_libusb_open failed");
            libusb_close(ov_usb_handle);
        }
    }
    PHONEAR_LOGI("HidSensor ov opened ? %d", ov_opened);
    return ov_opened;
}

void HidSensor::close() {
    std::unique_lock<std::mutex> lock(m_Mutex);
    internel_close(0);
}

void HidSensor::exit() {
    std::unique_lock<std::mutex> lock(m_Mutex);
    internel_exit(0);
}

int HidSensor::startReading(unsigned char *buffer, int length, unsigned int timeout) {
    int res = -1, size;
    if (ov_opened && (ov_usb_handle != nullptr)) {
        res = libusb_bulk_transfer(ov_usb_handle, 0x89, buffer, length, &size, timeout);
        if (res < 0) {
            PHONEAR_LOGE("libusb_bulk_transfer failed %d\n", res);
            if (res == LIBUSB_ERROR_PIPE) {
                res = libusb_clear_halt(ov_usb_handle, 0x89);
                PHONEAR_LOGE("libusb_clear_halt res = %d\n", res);
            }
        }
    }
    return res;
}

int HidSensor::startReading2(unsigned char *buffer, int length, unsigned int timeout) {
    int res = -1, size;
    if (ov_opened && (ov_usb_handle != nullptr)) {
        res = libusb_bulk_transfer(ov_usb_handle, 0x89, buffer, length, &size, timeout);
        if (res < 0) {
            PHONEAR_LOGE("libusb_bulk_transfer failed %d\n", res);
        }
    }
    return res;
}

int HidSensor::startReadingCaliData(int type, std::array<float, 3> &cali_data) {
    int res = -1, size;
    unsigned char buffer[32];
    float usb_accel_data[3] = {0};
    float usb_gyro_data[3] = {0};
    float acc_temp_data[3] = {0};
    float gyro_temp_data[3] = {0};
    Vector3f data;
    if (ov_opened && (ov_usb_handle != nullptr)) {
        res = libusb_bulk_transfer(ov_usb_handle, 0x89, buffer, 32, &size, 100);
        if (res < 0) {
            PHONEAR_LOGE("libusb_bulk_transfer failed %d\n", res);
            return res;
        }
        if (type == 0) {
            // get accel data
            usb_accel_data[0] =
                    (int16_t) (((buffer[13] << 8) & 0xFF00) | buffer[14]) *
                    ICM206XX_ACCEL_RESOLUTION_16G * G / 1000;
            usb_accel_data[1] =
                    (int16_t) (((buffer[15] << 8) & 0xFF00) | buffer[16]) *
                    ICM206XX_ACCEL_RESOLUTION_16G * G / 1000;
            usb_accel_data[2] =
                    (int16_t) (((buffer[17] << 8) & 0xFF00) | buffer[18]) *
                    ICM206XX_ACCEL_RESOLUTION_16G * G / 1000;

            AccelMatrix = Matrix3f(R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2],
                    R[2][0], R[2][1], R[2][2]);
            convertAxis(b_axis, acc_temp_data, usb_accel_data);
            data = {acc_temp_data[0], acc_temp_data[1], acc_temp_data[2]};
            data = AccelMatrix.Transform(data - AccelOffset);
            convertAxis(a_axis, acc_temp_data, (float*)&data);
            data = {acc_temp_data[0], acc_temp_data[1], acc_temp_data[2]};
            cali_data[0] = data[0];
            cali_data[1] = data[1];
            cali_data[2] = data[2];
        } else if (type == 1) {
            // get gyro data
            usb_gyro_data[0] =
                    (int16_t) (((buffer[21] << 8) & 0xFF00) | buffer[22]) *
                    ICM206XX_GYRO_SSTVT_2000DPS * PI / 180;
            usb_gyro_data[1] =
                    (int16_t) (((buffer[23] << 8) & 0xFF00) | buffer[24]) *
                    ICM206XX_GYRO_SSTVT_2000DPS * PI / 180;
            usb_gyro_data[2] =
                    (int16_t) (((buffer[25] << 8) & 0xFF00) | buffer[26]) *
                    ICM206XX_GYRO_SSTVT_2000DPS * PI / 180;

            GyroMatrix = Matrix3f(R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0],
                    R[2][1], R[2][2]);
            convertAxis(b_axis, gyro_temp_data, usb_gyro_data);
            data = {gyro_temp_data[0], gyro_temp_data[1], gyro_temp_data[2]};
            data = AccelMatrix.Transform(data - GyroOffset);
            convertAxis(a_axis, gyro_temp_data, (float*)&data);
            data = {gyro_temp_data[0], gyro_temp_data[1], gyro_temp_data[2]};
            cali_data[0] = data[0];
            cali_data[1] = data[1];
            cali_data[2] = data[2];

        } else {
            PHONEAR_LOGE("type is not supported %d\n", type);
            res = -2;
            return res;
        }
    }
    return res;
}

uint64_t HidSensor::getTimeNano()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t result = t.tv_sec * 1000000000LL + t.tv_nsec;
    return result;
}

int HidSensor::convertAxis(const int axis[3][3], float out[3], float in[3]) {
    int ret = 0;
    for (int row = 0; row < 3; ++row) {
        out[row] = 0.0;
        for (int col = 0; col < 3; ++col) {
            out[row] += axis[row][col]*in[col];
        }
    }
    return ret;
}

// add for ST usb control
int HidSensor::setStUfd(int vid, int pid, int fd, int busnum, int devaddr, const char *usbfs) {
    if (st_opened) {
        //check whether usbfs is the same
        if ((mStUsbfs != nullptr) && (strcmp(mStUsbfs, usbfs) == 0) && (mStFd == fd)
            && (mStBusnum == busnum) && (mStDevaddr == devaddr)) {
            PHONEAR_LOGE("setStUfd repeat, ignore current = %s, %d, %d, %d, new = %s, %d, %d, %d",
                         mStUsbfs, mStFd, mStBusnum, mStDevaddr, usbfs, fd, busnum, devaddr);
            return LIBUSB_ERROR_BUSY;
        } else {
            // close first
            PHONEAR_LOGE("setStUfd close previous device first");
            closeSt();

        }
    }
    mStVid = vid;
    mStPid = pid;
    mStFd = fd;
    mStBusnum = busnum;
    mStDevaddr = devaddr;
    if (mStUsbfs != nullptr) {
        free(mStUsbfs);
    }
    mStUsbfs = strdup(usbfs);
    PHONEAR_LOGI("setStUfd: new stusbfs = %s, stFd = %d", mStUsbfs, mStFd);
    return LIBUSB_SUCCESS;
}

int HidSensor::initSt(void) {
    int ret;
    if (mStUsbfs && strlen(mStUsbfs) > 0) {
        if (st_usb_ctx != nullptr) {
            PHONEAR_LOGE("init again, exit previous st_usb_ctx = %p", st_usb_ctx);
            libusb_exit(st_usb_ctx);
            st_usb_ctx = nullptr;
        }
        ret = libusb_init2(&st_usb_ctx, mStUsbfs);
        if (ret == LIBUSB_SUCCESS) {
            PHONEAR_LOGI("st libusb_init success = %p", st_usb_ctx);
            int ret = libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG);
            if (!ret) {
                PHONEAR_LOGE("st Hotplug capabilites are not supported on this platform");
            } else {
                PHONEAR_LOGI("st libusb_has_capability = %d", ret);
                // register hotplug callback
                if (st_monitor_running == 0) {
                    pthread_attr_init (&st_attr);
                    pthread_attr_setdetachstate(&st_attr, PTHREAD_CREATE_DETACHED);
                    pthread_create(&st_hotplug_t, &st_attr, st_hotplug_monitor_thread, NULL);
                }
            }
            return LIBUSB_SUCCESS;
        } else {
            PHONEAR_LOGE("st libusb_init failed");
            return LIBUSB_ERROR_NO_DEVICE;
        }
    } else {
        PHONEAR_LOGE("st usbfs incorrect");
        return LIBUSB_ERROR_INVALID_PARAM;
    }
}

int HidSensor::openSt(void) {
    int ret;
    st_usb_dev = libusb_get_device_with_fd(st_usb_ctx, mStVid, mStPid, nullptr, mStFd, mStBusnum,
                                           mStDevaddr);
    if (st_usb_dev == nullptr) {
        return LIBUSB_ERROR_ACCESS;
    }
    ret = libusb_open(st_usb_dev, &st_usb_handle);
    if (ret == LIBUSB_SUCCESS) {
        PHONEAR_LOGI("st libusb_open success");
        st_opened = true;
    } else {
        PHONEAR_LOGE("st libusb_open failed");
        libusb_close(st_usb_handle);
        return LIBUSB_ERROR_NO_DEVICE;
    }
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(st_usb_dev, &desc);
    ret = libusb_get_active_config_descriptor(st_usb_dev, &st_conf_desc);
    if (ret < 0)
        ret = libusb_get_config_descriptor(st_usb_dev, 0, &st_conf_desc);
    if (ret == LIBUSB_SUCCESS) {
        PHONEAR_LOGI("st libusb_get_config_descriptor success");
    } else {
        PHONEAR_LOGE("libusb_get_config_descriptor failed");
        libusb_free_config_descriptor(st_conf_desc);
        return LIBUSB_ERROR_NOT_SUPPORTED;
    }
    if (st_conf_desc) {
        int mInf;
        for (int j = 0; j < st_conf_desc->bNumInterfaces; j++) {
            const struct libusb_interface *intf = &st_conf_desc->interface[j];
            for (int k = 0; k < intf->num_altsetting; k++) {
                const struct libusb_interface_descriptor *intf_desc;
                intf_desc = &intf->altsetting[k];
                if (intf_desc->bInterfaceClass == LIBUSB_CLASS_HID) {
                    mInf = intf_desc->bInterfaceNumber;
                    PHONEAR_LOGI("st_inf_num is %d", mInf);
                    st_inf_num = -1;
                    if (mInf == kStInf) {
                        ret = libusb_set_auto_detach_kernel_driver(st_usb_handle, 1);
                        PHONEAR_LOGI("ST libusb_set_auto_detach_kernel_driver %d", ret);
                        ret = libusb_claim_interface(st_usb_handle, mInf);
                        if (ret >= 0) {
                            PHONEAR_LOGI("st_inf_num %d claim successfully", mInf);
                            st_inf_num = mInf;
                            /* Find the INPUT and OUTPUT endpoints. An OUTPUT endpoint is not required. */
                            for (int i = 0; i < intf_desc->bNumEndpoints; i++) {
                                const struct libusb_endpoint_descriptor *ep
                                        = &intf_desc->endpoint[i];

                                /* Determine the type and direction of this
                                   endpoint. */
                                int is_interrupt =
                                        (ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
                                        == LIBUSB_TRANSFER_TYPE_INTERRUPT;
                                int is_output =
                                        (ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK)
                                        == LIBUSB_ENDPOINT_OUT;
                                /* int is_input =
                                        (ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK)
                                        == LIBUSB_ENDPOINT_IN; */

                                /* Decide whether to use it for output. */
                                if (st_output_endpoint <= 0 &&
                                    is_interrupt && is_output) {
                                    /* Use this endpoint for OUTPUT */
                                    st_output_endpoint = ep->bEndpointAddress;
                                    PHONEAR_LOGI("st_output_endpoint = %d", st_output_endpoint);
                                }
                            }
                        } else {
                            libusb_release_interface(st_usb_handle, mInf);
                            PHONEAR_LOGE("libusb_claim_interface failed");
                            return LIBUSB_ERROR_BUSY;
                        }
                        return LIBUSB_SUCCESS;
                    } else {
                        PHONEAR_LOGE("libusb_claim_interface missing, need initialization again");
                        return LIBUSB_ERROR_PIPE;
                    }
                } else {
                    continue;
                }
            } /* find class HID */
        }
    } else {
        PHONEAR_LOGE("st_conf_desc failed");
        return LIBUSB_ERROR_IO;
    }
}

int HidSensor::handShakeWithSt(void) {
    PHONEAR_LOGI("handShakeWithSt");
    int ret;
    unsigned char sendStr[7];
    unsigned char readStr[7];
    int report_number;

    ret = initSt();
    if (ret != LIBUSB_SUCCESS)
        return ret;

    ret = openSt();
    if (ret != LIBUSB_SUCCESS)
        return ret;

    do {
        memset(sendStr, 0, sizeof(sendStr));
        sendStr[0] = 0x2;
        sendStr[1] = 0x21; //"!"
        sendStr[2] = 0x22; //"""
        sendStr[3] = 0x23; //"#"
        sendStr[4] = 0x24; //"$"
        sendStr[5] = 0x25; //"%"
        sendStr[6] = 0x26; //"&"
        // Send a Feature Report to the ST
        report_number = sendStr[0];
        ret = libusb_control_transfer(st_usb_handle,
                                      LIBUSB_REQUEST_TYPE_CLASS |
                                      LIBUSB_RECIPIENT_INTERFACE |
                                      LIBUSB_ENDPOINT_OUT, /*should be 21*/
                                      0x09/*HID set_report*/,
                                      (3/*HID feature*/ << 8) | report_number,
                                      st_inf_num,
                                      (unsigned char *) sendStr,
                                      sizeof(sendStr),
                                      1000/*timeout millis*/);
        PHONEAR_LOGI("send a feature report to ST %d", ret);
        if (ret == LIBUSB_ERROR_PIPE)
            libusb_clear_halt(st_usb_handle, 0);

        // Read a Feature Report from the ST
        memset(readStr, 0, sizeof(readStr));
        readStr[0] = 0x2;
        report_number = readStr[0];

        ret = libusb_control_transfer(st_usb_handle,
                                      LIBUSB_REQUEST_TYPE_CLASS |
                                      LIBUSB_RECIPIENT_INTERFACE |
                                      LIBUSB_ENDPOINT_IN, /*should be A1*/
                                      0x01/*HID get_report*/,
                                      (3/*HID feature*/ << 8) | report_number,
                                      st_inf_num,
                                      (unsigned char *) readStr,
                                      sizeof(readStr),
                                      1000/*timeout millis*/);
        PHONEAR_LOGI("Read a feature report from ST %d", ret);
        if (ret == LIBUSB_ERROR_PIPE)
            libusb_clear_halt(st_usb_handle, 0);

        //compare send and read str
        if (strncmp((const char *) sendStr, (const char *) readStr,
                    sizeof(sendStr)) == 0) {
            PHONEAR_LOGI("compare pass");
        }

    } while (ret <= 0);

    //for (i = 0; i < ret; i++) {
    //   PHONEAR_LOGI("readstr[%d] = %02hhx ", i, readStr[i]);
    //}
    return LIBUSB_SUCCESS;
}

int HidSensor::sendCommandToSt(unsigned char command, unsigned char value) {
    int ret;
    unsigned char sendStr[3];
    unsigned char readStr[3];
    int report_number;
    if (command == 0x3) {
        if (value > 0x9) {
            value = 0x9;
        }
    }
    memset(sendStr, 0, sizeof(sendStr));
    sendStr[0] = 0x2;
    sendStr[1] = command;
    sendStr[2] = value;
    if (st_opened == false) {
        ret = initSt();
        if (ret != LIBUSB_SUCCESS)
            return ret;

        ret = openSt();
        if (ret != LIBUSB_SUCCESS)
            return ret;
    }
    // Send a Command to the ST
    report_number = sendStr[0];
    ret = libusb_control_transfer(st_usb_handle,
                                  LIBUSB_REQUEST_TYPE_CLASS |
                                  LIBUSB_RECIPIENT_INTERFACE |
                                  LIBUSB_ENDPOINT_OUT, /*should be 21*/
                                  0x09/*HID set_report*/,
                                  (3/*HID feature*/ << 8) | report_number,
                                  st_inf_num,
                                  (unsigned char *) sendStr,
                                  sizeof(sendStr),
                                  1000/*timeout millis*/);
    PHONEAR_LOGI("sendCommandToSt command = %x, value = %x ret = %d", command, value, ret);
    return ret;
}

long long GetMonotonicTimeNano() {
    struct timespec res;
    clock_gettime(CLOCK_MONOTONIC, &res);
    return (res.tv_sec * kNanosInSeconds) + res.tv_nsec;
}

int HidSensor::sendDataToSt(int x, int y) {
    int ret;
    int i;
    long long sendTimes = 0;
    if (st_opened == false) {
        PHONEAR_LOGI("open device please. do open first");
        ret = openSt();
        if (ret == LIBUSB_SUCCESS) {
            PHONEAR_LOGI("open device successfully, go ahead");
        } else {
            PHONEAR_LOGE("open device please");
            return LIBUSB_ERROR_NO_DEVICE;
        }
    }
    if (st_opened) {
        if (st_inf_num == kStInf) {
            if (st_output_endpoint > 0) {
                unsigned char buf[256];
                memset(buf, 0, sizeof(buf));
                buf[0] = 0x1; //The first byte is the report number (0x1).
                for (i = 0; i < 4; i++) {
                    buf[i + 1] = (unsigned char) (x >> (24 - i * 8));
                    buf[i + 5] = (unsigned char) (y >> (24 - i * 8));
                    if (ST_LOG_DEBUG) {
                        PHONEAR_LOGI("buf[%d] = %02hhx, buf[%d] = %02hhx", i + 1, buf[i + 1], i + 5,
                                     buf[i + 5]);
                    }
                }
                int actual_length = 0;
                ret = libusb_interrupt_transfer(st_usb_handle,
                                                st_output_endpoint,
                                                (unsigned char *) buf,
                                                10,
                                                &actual_length, 20);
                sendTimes = GetMonotonicTimeNano();
                if (ret == 0) {
                    if (ST_LOG_DEBUG) {
                        PHONEAR_LOGI(
                                "SEND_DATA_TO_ST x = %d, y = %d, ret = %d, actual_length = %d, sendTime = %lld",
                                x, y, ret, actual_length, sendTimes);
                    }
                } else {
                    PHONEAR_LOGE("SEND_DATA_TO_ST failed %d", ret);
                }

                if ((ret == 0) && (actual_length > 0)) {
                    return LIBUSB_SUCCESS;
                }
                return LIBUSB_ERROR_IO;
            } else {
                PHONEAR_LOGE("st_output_endpoint missing");
                return LIBUSB_ERROR_PIPE;
            }
        } else {
            PHONEAR_LOGE("libusb_claim_interface missing, can't send data");
            return LIBUSB_ERROR_PIPE;
        }
    } else {
        PHONEAR_LOGE("open device please");
        return LIBUSB_ERROR_NO_DEVICE;
    }
}

void* caliFile_write(void *arg) {
    FILE *fstream = NULL;
    int start_len = 9;
    unsigned char buf_begin[9];
    int i;
    int real_data_len = 0;

    memset(buf_begin, 0, sizeof(buf_begin));

    buf_begin[0] = 0x66; //f
    buf_begin[1] = 0x69; //i
    buf_begin[2] = 0x6c; //l
    buf_begin[3] = 0x65; //e
    buf_begin[4] = 0x62; //b
    buf_begin[5] = 0x65; //e
    buf_begin[6] = 0x67; //g
    buf_begin[7] = 0x69; //i
    buf_begin[8] = 0x6e; //n

    //check kCaliBuf valid
    if (strncmp((const char *) kCaliBuf, (const char *) buf_begin, start_len) == 0) {
        PHONEAR_LOGI("kCaliBuf is valid, find file begin");
    } else {
        PHONEAR_LOGE("kCaliBuf is invalid, can't find file begin");
        return NULL;
    }

    //find buf end, get the real data length
    for (i = start_len; i < CALI_FILE_LEN; i++) {
        // PHONEAR_LOGI("kCaliBuf[%d] = %02hhx", i, kCaliBuf[i]);
        if ((kCaliBuf[i] == 0x66) && (i < (CALI_FILE_LEN - 6))) {
            if ((kCaliBuf[i + 1] == 0x69) &&
                (kCaliBuf[i + 2] == 0x6c) &&
                (kCaliBuf[i + 3] == 0x65) &&
                (kCaliBuf[i + 4] == 0x65) &&
                (kCaliBuf[i + 5] == 0x6e) &&
                (kCaliBuf[i + 6] == 0x64)) {
                real_data_len = i;
                PHONEAR_LOGI("kCaliBuf is valid, find file end, real_data_len = %d", real_data_len);
                break;
            }
        } else {
            continue;
        }
    }

    if (real_data_len == 0) {
        PHONEAR_LOGE("kCaliBuf is invalid, can't find file end");
        return NULL;
    }

    fstream = fopen(FILE_NAME, "w+");
    if (fstream == NULL) {
        PHONEAR_LOGE("open file %s failed", FILE_NAME);
        return NULL;
    }

    fwrite(kCaliBuf + start_len, real_data_len - start_len, sizeof(char), fstream);
    fclose(fstream);
    PHONEAR_LOGI("calibration fwrite completed");

    //uzip Calibration files
    char cmd[128];
    sprintf(cmd, "tar xzvf %s -C /sdcard/DCIM", FILE_NAME);
    system(cmd);
    PHONEAR_LOGI("caliFile_write exit");
    return NULL;
}

int HidSensor::receiveDataFromStControlChannel(void) {
    int ret;
    int i;
    int count = 0;
    int report_number;
    int checkEnd = 0;
    if (st_opened) {
        if (st_inf_num == kStInf) {
            unsigned char sendbuf[2];
            unsigned char recbuf[256];
            memset(sendbuf, 0, sizeof(sendbuf));
            memset(recbuf, 0, sizeof(recbuf));
            memset(kCaliBuf, 0, sizeof(kCaliBuf));
            do {
                memset(sendbuf, 0, sizeof(sendbuf));
                sendbuf[0] = 0x10 + count;
                report_number = sendbuf[0];
                ret = libusb_control_transfer(st_usb_handle,
                                              LIBUSB_REQUEST_TYPE_CLASS |
                                              LIBUSB_RECIPIENT_INTERFACE |
                                              LIBUSB_ENDPOINT_OUT, /*should be 21*/
                                              0x09/*HID set_report*/,
                                              (3/*HID feature*/ << 8 | report_number),
                                              st_inf_num,
                                              (unsigned char *) sendbuf,
                                              sizeof(sendbuf),
                                              1000/*timeout millis*/);
                if (ret > 0) {
                    if (ST_LOG_DEBUG) {
                        PHONEAR_LOGI("send a read[%d] buf[0] = %02hhx request to ST %d", count,
                                     sendbuf[0], ret);
                    }
                } else {
                    PHONEAR_LOGE("st set report failed %d", ret);
                }

                // Read a Feature Report from the ST
                memset(recbuf, 0, sizeof(recbuf));
                recbuf[0] = 0x10 + count;
                report_number = recbuf[0];
                ret = libusb_control_transfer(st_usb_handle,
                                              LIBUSB_REQUEST_TYPE_CLASS |
                                              LIBUSB_RECIPIENT_INTERFACE |
                                              LIBUSB_ENDPOINT_IN, /*should be A1*/
                                              0x01/*HID get_report*/,
                                              (3/*HID feature*/ << 8 | report_number),
                                              st_inf_num,
                                              (unsigned char *) recbuf,
                                              sizeof(recbuf),
                                              1000/*timeout millis*/);
                if (ret > 0) {
                    if (ST_LOG_DEBUG) {
                        PHONEAR_LOGI("the %d data received from ST,  actual length is %d", count,
                                     ret);
                        for (i = 0; i < ret; i++) {
                            PHONEAR_LOGI("buf[%d-%d] = %02hhx ", count, i, recbuf[i]);
                        }
                    }
                } else {
                    PHONEAR_LOGE("st get report failed %d", ret);
                }

                if (recbuf[0] == 0xaa) {
                    for (i = 1; i < 7; i++) {
                        if (recbuf[i] != 0xaa) {
                            checkEnd = 0;
                            PHONEAR_LOGI("the data not end package %d", count);
                            memcpy(kCaliBuf + (count * ret), recbuf, ret);
                            count++;
                            break;
                        } else {
                            if ((i == 6) && (recbuf[i] == 0xaa)) {
                                checkEnd = 1;
                                PHONEAR_LOGI("the data end package %d", count);
                            }
                        }
                    }
                } else {
                    // copy data to kCaliBuf
                    if (count * ret > CALI_FILE_LEN) {
                        PHONEAR_LOGE("receive calibration buffer is overflow");
                        return LIBUSB_ERROR_OVERFLOW;
                    }
                    memcpy(kCaliBuf + (count * ret), recbuf, ret);
                    if (ST_LOG_DEBUG) {
                        for (i = 0; i < ret; i++) {
                            PHONEAR_LOGI("kCaliBuf[%d-%d] = %02hhx ", count * ret, i,
                                         kCaliBuf[count * ret + i]);
                        }
                    }
                    count++;
                }
            } while ((checkEnd == 0) && (count < 80));
            PHONEAR_LOGI("the %d data received from ST end", count);

            //start a thread to save calibration data to file
            pthread_create(&file_write_t, NULL, caliFile_write, NULL);

            return LIBUSB_SUCCESS;
        } else {
            PHONEAR_LOGE("libusb_claim_interface missing, can't send data");
            return LIBUSB_ERROR_PIPE;
        }
    } else {
        PHONEAR_LOGE("open device please");
        return LIBUSB_ERROR_NO_DEVICE;
    }
}

int HidSensor::receiveDataFromStDataChannel(void) {
    int ret;
    int i;
    long long receiveTimes = 0;
    if (st_opened) {
        if (st_inf_num == kStInf) {
            if (st_input_endpoint > 0) {
                unsigned char buf[256];
                memset(buf, 0, sizeof(buf));
                buf[0] = 0x1; //The first byte is the report number (0x1).
                int actual_length = 0;
                ret = libusb_interrupt_transfer(st_usb_handle,
                                                st_input_endpoint,
                                                (unsigned char *) buf,
                                                64,
                                                &actual_length, 20);
                receiveTimes = GetMonotonicTimeNano();
                PHONEAR_LOGI(
                        "RECEIVE Data from input endpoint ret = %d, actual_length = %d, sendTime = %lld",
                        ret, actual_length, receiveTimes);

                if ((ret == 0) && (actual_length > 0)) {
                    for (i = 0; i < actual_length; i++) {
                        PHONEAR_LOGI("buf[%d] = %02hhx ", i, buf[i]);
                    }
                    return LIBUSB_SUCCESS;
                }
                return LIBUSB_ERROR_IO;
            } else {
                PHONEAR_LOGE("st_input_endpoint missing");
                return LIBUSB_ERROR_PIPE;
            }
        } else {
            PHONEAR_LOGE("libusb_claim_interface missing, can't send data");
            return LIBUSB_ERROR_PIPE;
        }
    } else {
        PHONEAR_LOGE("open device please");
        return LIBUSB_ERROR_NO_DEVICE;
    }
}

void HidSensor::closeSt(void) {
    std::unique_lock<std::mutex> lock(m_Mutex);
    internel_close(1);
}

void HidSensor::exitSt(void) {
    std::unique_lock<std::mutex> lock(m_Mutex);
    internel_exit(1);
}
} /*namespace cardboard*/
