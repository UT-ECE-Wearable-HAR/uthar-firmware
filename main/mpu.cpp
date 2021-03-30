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

Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]
volatile double ypr_data[3];
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[PACKET_SIZE * PACKET_BUF_LEN + 64]; // FIFO storage buffer
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
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

void mpu_read() {
  int64_t before_read = esp_timer_get_time();
  const TickType_t delay = pdMS_TO_TICKS(5);
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
  }
  // wait for DMP data ready interrupt frequently)
  while (!(mpuIntStatus & 0x02)) {
    vTaskDelay(delay);
  }
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < (PACKET_SIZE * PACKET_BUF_LEN)) {
    vTaskDelay(delay);
    fifoCount = mpu.getFIFOCount();
  }

  // read a packet from FIFO
  for (uint8_t i = 0; i < PACKET_BUF_LEN; i++) {
    mpu.getFIFOBytes(fifoBuffer + i * PACKET_SIZE, PACKET_SIZE);
  }
  // empty fifo
  mpu.resetFIFO();
  int64_t frame_time = (esp_timer_get_time() - before_read) / 1000;
  ESP_LOGI("stream", "read mpu: %" PRId64 "ms", frame_time);
}
}
