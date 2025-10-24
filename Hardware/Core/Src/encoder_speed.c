#include "encoder_speed.h"
#include <stdio.h>
#include <string.h>

// 采样周期（毫秒）
#define SAMPLE_TIME_MS 10.0f

// 初始化编码器数据测量
void Encoder_Data_Init(void)
{

}

// 更新编码器数据测量
void Encoder_Data_Update(Encoder_Data_t *encoder_data)
{
    // 获取左右轮脉冲差值（用于速度计算）
    int16_t left_pulse_diff = Encoder_GetLeft();
    int16_t right_pulse_diff = Encoder_GetRight();
    
    // 计算左右轮速度（厘米/秒）
    encoder_data->left_speed_cm_s = Calculate_Wheel_Speed(left_pulse_diff, SAMPLE_TIME_MS);
    encoder_data->right_speed_cm_s = Calculate_Wheel_Speed(right_pulse_diff, SAMPLE_TIME_MS);
    
    // 获取左右轮累积计数（用于距离计算）
    int32_t left_total = Encoder_GetLeftCount();
    int32_t right_total = Encoder_GetRightCount();
    
    // 计算左右轮距离（厘米）
    encoder_data->left_distance_cm = Calculate_Wheel_Distance(left_total);
    encoder_data->right_distance_cm = Calculate_Wheel_Distance(right_total);
}

// 复位编码器数据测量
void Encoder_Data_Reset(Encoder_Data_t *encoder_data)
{
    encoder_data->left_speed_cm_s = 0.0f;
    encoder_data->right_speed_cm_s = 0.0f;
    encoder_data->left_distance_cm = 0.0f;
    encoder_data->right_distance_cm = 0.0f;
    
    // 复位编码器计数器
    Encoder_Reset();
}

// 计算轮子速度（厘米/秒）
float Calculate_Wheel_Speed(int16_t pulse_diff, float sample_time_ms)
{
    // 速度 = 脉冲差值 / (编码器分辨率 * 采样时间) * 轮子周长
    float sample_time_s = sample_time_ms / 1000.0f;
    
    return (float)pulse_diff / (ENCODER_RESOLUTION * sample_time_s) * WHEEL_CIRCUMFERENCE_CM;
}

// 计算轮子距离（厘米）
float Calculate_Wheel_Distance(int32_t total_pulses)
{
    // 距离 = 累积脉冲数 / 编码器分辨率 * 轮子周长
    return (float)total_pulses / ENCODER_RESOLUTION * WHEEL_CIRCUMFERENCE_CM;
}

