#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx_hal.h"

// 编码器状态结构体
typedef struct {
    uint16_t last_counter;    // 上次计数器值
    int32_t total_count;      // 累积计数
} Encoder_State_t;

// 编码器计数器最大值（统一使用16位最大值）
#define ENCODER_MAX_COUNT  65535  // 统一使用16位最大值

void Encoder_Init(void);
int16_t Encoder_GetLeft(void);
int16_t Encoder_GetRight(void);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_Reset(void);
void Encoder_Get_Status(uint32_t *left_total, uint32_t *right_total);

#endif // __ENCODER_H__ 