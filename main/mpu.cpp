#include "mpu.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PIN_SDA 22
#define PIN_CLK 23
#define PACKET_BUF_LEN 10

Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]
volatile double ypr_data[3];
uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
MPU6050 mpu;

extern "C" {

void task_initI2C(void *ignore) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)PIN_SDA;
  conf.scl_io_num = (gpio_num_t)PIN_CLK;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  vTaskDelete(NULL);
}

void mpu_init(void) {
  mpu = MPU6050();
  mpu.initialize();
  mpu.dmpInitialize();

  // This need to be setup individually
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  mpu.setDMPEnabled(true);
}

uint8_t mpu_read(uint8_t *packet) {
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(packet, packetSize);
    return 0;
  }
  return 1;
}

uint8_t packetBuf[42 * PACKET_BUF_LEN + 64];
uint8_t packetBufCpy[42 * PACKET_BUF_LEN + 64];
TaskHandle_t xTaskToNotify = NULL;

void mpu_task(void *) {
  for (uint8_t i = 0; i < PACKET_BUF_LEN; i++) {
    while (mpu_read(packetBuf + 42 * i)) {
    }
    vTaskDelay(pdMS_TO_TICKS(67));
  }
  memcpy(packetBuf, packetBufCpy, 42 * PACKET_BUF_LEN + 64);
  xTaskNotifyGive(xTaskToNotify);
  vTaskDelete(NULL);
}
}
