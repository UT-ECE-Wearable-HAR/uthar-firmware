#ifndef __MPU_H_
#define __MPU_H_

#ifdef __cplusplus
extern "C" {
#endif

void task_initI2C(void *);
void task_mpu6050(void *);

#ifdef __cplusplus
}
#endif

#endif // __MPU_H_
