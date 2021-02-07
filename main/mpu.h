#ifndef __MPU_H_
#define __MPU_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_SIZE 42

void task_initI2C(void *);
void task_mpu6050(void *);
void mpu_init(void);
uint8_t mpu_read(uint8_t *packet);

#ifdef __cplusplus
}
#endif

#endif // __MPU_H_
