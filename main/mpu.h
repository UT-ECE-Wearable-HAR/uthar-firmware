#ifndef __MPU_H_
#define __MPU_H_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_SIZE 42
#define PACKET_BUF_LEN 10

extern uint8_t packetBuf[PACKET_SIZE*PACKET_BUF_LEN+64];
extern TaskHandle_t xTaskToNotify;

void task_initI2C(void *);
void task_mpu6050(void *);
void mpu_init(void);
uint8_t mpu_read(uint8_t *packet);
void mpu_task(void *);

#ifdef __cplusplus
}
#endif

#endif // __MPU_H_
