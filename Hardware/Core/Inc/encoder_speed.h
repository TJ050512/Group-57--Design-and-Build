#ifndef __ENCODER_SPEED_H__
#define __ENCODER_SPEED_H__

#include "stm32f4xx_hal.h"
#include "encoder.h"

// 编码器参数定义 - 根据MC520电机参数调整
#define ENCODER_PPR          390     // 编码器物理分辨率（脉冲/圈）- MC520: 13线×30减速比=390
#define ENCODER_RESOLUTION   (ENCODER_PPR * 4)  // 实际分辨率（A、B相4倍频）= 1560脉冲/圈
#define WHEEL_DIAMETER_MM    65      // 轮子直径（毫米）- 请确认实际轮子尺寸
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159f)  // 轮子周长（毫米）≈ 204.2mm
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_CIRCUMFERENCE_MM / 10.0f)  // 轮子周长（厘米）≈ 20.42cm

// 编码器速度结构体
typedef struct {
    float left_speed_cm_s;     // 左轮速度（厘米/秒）
    float right_speed_cm_s;    // 右轮速度（厘米/秒）
    float left_distance_cm;    // 左轮累计距离（厘米）
    float right_distance_cm;   // 右轮累计距离（厘米）
} Encoder_Data_t;

// 函数声明
void Encoder_Data_Init(void);
void Encoder_Data_Update(Encoder_Data_t *encoder_data);
void Encoder_Data_Reset(Encoder_Data_t *encoder_data);

// 速度计算函数
float Calculate_Wheel_Speed(int16_t pulse_diff, float sample_time_ms);  // 返回厘米/秒
float Calculate_Wheel_Distance(int32_t total_pulses);  // 返回厘米

#endif // __ENCODER_SPEED_H__ 