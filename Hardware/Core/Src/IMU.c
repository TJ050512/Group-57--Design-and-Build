#include "IMU.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"  // ?? I2C ?
#include "usart.h" // ?? USART ?
#include <stdio.h>
#include <string.h>

void IMU_Init(void) {
    uint8_t data[2];
    // ??MPU6500,?????
    data[0] = 0x6B; data[1] = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR << 1, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    // ??????? ±250dps
    data[0] = 0x1B; data[1] = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR << 1, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    // ??????? 5Hz
    data[0] = 0x1A; data[1] = 0x06;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR << 1, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    // ??????? 100Hz
    data[0] = 0x19; data[1] = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR << 1, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
    HAL_Delay(50); // ??????
    // ??WHO_AM_I???,???
    uint8_t who_am_i = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR << 1, 0x75, 1, &who_am_i, 1, HAL_MAX_DELAY);
    char msg[32];
    sprintf(msg, "WHO_AM_I: 0x%02X\n", who_am_i);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void IMU_InitYawReference(SensorData* data) {
    data->yaw = 0.0f;  // ??????0°,???????????
}

void IMU_CalibrateGyroZ(SensorData* data) {
    float sum = 0.0f;
    uint16_t count = 100; // ??100?
    
    // ??????
    HAL_UART_Transmit(&huart3, (uint8_t*)"Calibrating gyro Z... Keep still!\n", 36, 100);
    
    // ??100?????,?????
    for (uint16_t i = 0; i < count; i++) {
        IMU_ReadSensor(data);
        sum += data->gyroZ; // ?????
        HAL_Delay(10); // ?10ms????
    }
    
    // ???????(??????°/s)
    data->gyroZ_offset = (sum / count) / 131.0f;
    data->is_calibrated = 1; // ??????
    
    // ??????
    char msg[50];
    sprintf(msg, "Gyro Z offset: %.4f °/s\n", data->gyroZ_offset);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

// ??:????(?????Z???,??0~360°)
void IMU_UpdateYaw(SensorData* data, float dt) {
    // 1. ???Z?????????(°/s),?????
    float gyroZ_deg_s = (data->gyroZ / 131.0f) - data->gyroZ_offset;
    gyroZ_deg_s = -gyroZ_deg_s;
    // 2. ????,???????(??????)
    if (!data->is_calibrated) {
        return;
    }
    
    // 3. ??????(??????)
    data->yaw += gyroZ_deg_s * dt; // ?????????
    
    // 4. ??????(??)
    if (data->yaw >= 360.0f) {
        data->yaw -= 360.0f;
    } else if (data->yaw < 0.0f) {
        data->yaw += 360.0f;
    }
}

void IMU_ReadSensor(SensorData* data) {
    uint8_t buffer[14];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR << 1, 0x3B, 1, buffer, 14, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        char err[] = "I2C Read Error!\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
        return;
    }

    // ????????????
    data->accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temperature = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
}

void IMU_PrintRawData(const SensorData* data) {
    // ?? HAL_UART_Transmit ?? printf ??????
    uint8_t txt[] = "=== Raw Data ===\n";
    HAL_UART_Transmit(&huart3, txt, sizeof(txt) - 1, 100);  // ???Raw Data???
    char buffer[50];
    sprintf(buffer, "Accel (X,Y,Z): %d, %d, %d\n", data->accelX, data->accelY, data->accelZ);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);  // ???????
    sprintf(buffer, "Gyro (X,Y,Z): %d, %d, %d\n", data->gyroX, data->gyroY, data->gyroZ);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);  // ???????
}

void IMU_PrintPhysicalValues(const SensorData* data) {
    // ?? HAL_UART_Transmit ?? printf ??????
    uint8_t txt[] = "=== Physical Values ===\n";
    HAL_UART_Transmit(&huart3, txt, sizeof(txt) - 1, 100);  // ???Physical Values???
    char buffer[50];
    sprintf(buffer, "Accel (g): %.4f, %.4f, %.4f\n", data->accelX / 16384.0, data->accelY / 16384.0, data->accelZ / 16384.0);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);  // ???????(g)
    sprintf(buffer, "Gyro (/s): %.4f, %.4f, %.4f\n", data->gyroX / 131.0, data->gyroY / 131.0, data->gyroZ / 131.0);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);  // ???????(?/s)
	  sprintf(buffer, "Current Angle: %.2f degrees\n", data->yaw);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

}
