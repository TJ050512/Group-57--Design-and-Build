#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"  // ????? STM32 ? HAL ???
#define MPU6500_ADDR 0x68    // MPU6500 ? I2C ??
#define SDA_PIN GPIO_PIN_9    // SDA ??
#define SCL_PIN GPIO_PIN_8    // SCL ??

// ?? SensorData ???????????
typedef struct {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temperature;
	  float yaw; 
	  float gyroZ_offset;  // ???Z??????
    uint8_t is_calibrated; // ??????(0=???,1=???)
} SensorData;

// IMU ?????
void IMU_Init(void);               // ???IMU
void IMU_ReadSensor(SensorData* data);  // ??IMU?????
void IMU_PrintRawData(const SensorData* data);  // ??????
void IMU_PrintPhysicalValues(const SensorData* data);  // ?????
void IMU_InitYawReference(SensorData* data);  // ??????0°
void IMU_UpdateYaw(SensorData* data, float dt);
void IMU_CalibrateGyroZ(SensorData* data);
#endif

