#include "encoder.h"
#include "tim.h"

// 左编码器用TIM4，右编码器用TIM2

// 编码器状态变量（私有）
static Encoder_State_t left_encoder = {0, 0};
static Encoder_State_t right_encoder = {0, 0};

void Encoder_Init(void)
{
    // 编码器模式初始化已由CubeMX生成的HAL库tim.c完成
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    
    // 初始化编码器状态
    left_encoder.last_counter = __HAL_TIM_GET_COUNTER(&htim4);
    right_encoder.last_counter = __HAL_TIM_GET_COUNTER(&htim2);
    left_encoder.total_count = 0;
    right_encoder.total_count = 0;
}

// 处理编码器溢出和累积计数
static void Encoder_Update_State(Encoder_State_t *encoder, uint16_t current_counter, uint16_t max_count)
{
    int32_t diff;
    
    // 计算计数器差值，正确处理正向和负向溢出
    if (current_counter >= encoder->last_counter) {
        // 正向计数
        diff = current_counter - encoder->last_counter;
        // 检查是否发生正向溢出（差值过大）
        if (diff > max_count / 2) {
            // 实际是负向溢出
            diff = diff - max_count - 1;
        }
    } else {
        // 发生溢出
        diff = current_counter - encoder->last_counter;
        // 检查是否发生负向溢出
        if (diff < -max_count / 2) {
            // 实际是正向溢出
            diff = diff + max_count + 1;
        }
    }
    
    // 更新累积计数
    encoder->total_count += diff;
    encoder->last_counter = current_counter;
}

int16_t Encoder_GetLeft(void)
{
    // 获取左编码器当前计数器值（用于速度计算）
    uint16_t current_counter = __HAL_TIM_GET_COUNTER(&htim4);
    
    // 计算本次的脉冲差值，使用改进的溢出处理
    int32_t diff;
    if (current_counter >= left_encoder.last_counter) {
        diff = current_counter - left_encoder.last_counter;
        // 检查是否发生正向溢出（差值过大）
        if (diff > ENCODER_MAX_COUNT / 2) {
            // 实际是负向溢出
            diff = diff - ENCODER_MAX_COUNT - 1;
        }
    } else {
        // 发生溢出
        diff = current_counter - left_encoder.last_counter;
        // 检查是否发生负向溢出
        if (diff < -ENCODER_MAX_COUNT / 2) {
            // 实际是正向溢出
            diff = diff + ENCODER_MAX_COUNT + 1;
        }
    }
    
    // 修复：更新last_counter用于下次速度计算
    //left_encoder.last_counter = current_counter;
    
    return (int16_t)diff;
}

int16_t Encoder_GetRight(void)
{
    // 获取右编码器当前计数器值（用于速度计算）
    uint16_t current_counter = __HAL_TIM_GET_COUNTER(&htim2);
    
    // 计算本次的脉冲差值，使用改进的溢出处理
    int32_t diff;
    if (current_counter >= right_encoder.last_counter) {
        diff = current_counter - right_encoder.last_counter;
        // 检查是否发生正向溢出（差值过大）
        if (diff > ENCODER_MAX_COUNT / 2) {
            // 实际是负向溢出
            diff = diff - ENCODER_MAX_COUNT - 1;
        }
    } else {
        // 发生溢出
        diff = current_counter - right_encoder.last_counter;
        // 检查是否发生负向溢出
        if (diff < -ENCODER_MAX_COUNT / 2) {
            // 实际是正向溢出
            diff = diff + ENCODER_MAX_COUNT + 1;
        }
    }
    
    // 修复：更新last_counter用于下次速度计算
   // right_encoder.last_counter = current_counter;
    
    return (int16_t)diff;
}

int32_t Encoder_GetLeftCount(void)
{
    // 获取左编码器累积计数（用于位置测量）
    uint16_t current_counter = __HAL_TIM_GET_COUNTER(&htim4);
    
    // 更新累积状态
    Encoder_Update_State(&left_encoder, current_counter, ENCODER_MAX_COUNT);
    
    return left_encoder.total_count;
}

int32_t Encoder_GetRightCount(void)
{
    // 获取右编码器累积计数（用于位置测量）
    uint16_t current_counter = __HAL_TIM_GET_COUNTER(&htim2);
    
    // 更新累积状态
    Encoder_Update_State(&right_encoder, current_counter, ENCODER_MAX_COUNT);
    
    return right_encoder.total_count;
}

void Encoder_Reset(void)
{
    // 复位硬件计数器
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    // 复位软件状态
    left_encoder.last_counter = 0;
    right_encoder.last_counter = 0;
    left_encoder.total_count = 0;
    right_encoder.total_count = 0;
}

// 新增：获取编码器状态信息
void Encoder_Get_Status(uint32_t *left_total, uint32_t *right_total)
{
    *left_total = left_encoder.total_count;
    *right_total = right_encoder.total_count;
} 