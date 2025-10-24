#include "motor.h"
#include "tim.h"
#include "PID.h"

#define PWM_MAX 999

// PID控制器实例 - 左右轮分别控制
PID_t pid_left = {0};
PID_t pid_right = {0};
volatile int motor_is_stop =1;
// PID初始化函数
void Motor_PID_Init(void)
{
    // 左轮PID参数设置 - 针对速度控制优化
    pid_left.Kp = 2.0f;         // 比例增益 - 提高响应速度
    pid_left.Ki = 0.8f;         // 积分增益 - 消除稳态误差
    pid_left.Kd = 0.3f;         // 微分增益 - 减少超调
    pid_left.OutMax = PWM_MAX;
    pid_left.OutMin = 0;
    pid_left.ErrorInt = 0;      // 初始化积分项
    
    // 右轮PID参数设置 - 针对速度控制优化
    pid_right.Kp = 2.0f;        // 比例增益 - 提高响应速度
    pid_right.Ki = 0.8f;        // 积分增益 - 消除稳态误差
    pid_right.Kd = 0.3f;        // 微分增益 - 减少超调
    pid_right.OutMax = PWM_MAX;
    pid_right.OutMin = 0;
    pid_right.ErrorInt = 0;     // 初始化积分项
}

// 速度�h��换系数配置 - 根据两轮小车B款实际规格调整
// 两轮小车B款实际规格: 速度1.2m/s = 120cm/s, 65mm轮胎
// 这是经过实际测试验证的速度规格
#define MAX_THEORETICAL_SPEED_CM_S 100.0f // 实际最大速度(cm/s)，基于两轮小车B款规格
#define SPEED_CALIBRATION_FACTOR 1.0f     // 校准系数，通过实际测试调整

// 更新PID实际速度值（从编码器数据获取）
void Motor_UpdateActualSpeed(float left_speed, float right_speed)
{
    // 应用校准系数到左右轮速度
    float calibrated_left_speed = left_speed * SPEED_CALIBRATION_FACTOR;
    float calibrated_right_speed = right_speed * SPEED_CALIBRATION_FACTOR;
    
    // ???????????????????????
    float speed_diff = calibrated_left_speed - calibrated_right_speed;
    if (speed_diff > 5.0f) {  // ????????5cm/s
        calibrated_right_speed += speed_diff * 0.3f;  // ????
    } else if (speed_diff < -5.0f) {  // ????????5cm/s
        calibrated_left_speed -= speed_diff * 0.3f;   // ????
    }
    
    // 将cm/s转换为PWM值 - 左轮
    float left_speed_ratio = calibrated_left_speed / MAX_THEORETICAL_SPEED_CM_S;
    uint16_t left_pwm_speed = (uint16_t)(left_speed_ratio * PWM_MAX);
    if (left_pwm_speed > PWM_MAX) left_pwm_speed = PWM_MAX;
    
    // 将cm/s转换为PWM值 - 右轮
    float right_speed_ratio = calibrated_right_speed / MAX_THEORETICAL_SPEED_CM_S;
    uint16_t right_pwm_speed = (uint16_t)(right_speed_ratio * PWM_MAX);
    if (right_pwm_speed > PWM_MAX) right_pwm_speed = PWM_MAX;
    
    // 更新左右轮PID控制器的实际值
    pid_left.Actual = (float)left_pwm_speed;
    pid_right.Actual = (float)right_pwm_speed;
}

void Motor_Init(void)
{
  // 启动PWM输出
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // 初始化PID控制器
  Motor_PID_Init();

  // 停止电机
  Motor_Stop();
}

void Motor_Stop(void)
{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    
    // 停止时重置PID控制器，避免积分饱和
    Motor_PID_Reset();
}

// 前进 - 两个电机都正转，使用左右轮独立PID控制
void Motor_Forward(uint16_t target_speed)
{
    if (target_speed > PWM_MAX) target_speed = PWM_MAX;
    
    // 设置左右轮PID目标值
    pid_left.Target = (float)target_speed;
    pid_right.Target = (float)target_speed;
    
    // 更新左右轮PID控制器
    PID_Update(&pid_left);
    PID_Update(&pid_right);
    
    // 获取左右轮PID输出值并限制范围
    uint16_t left_output = (uint16_t)pid_left.Out;
    uint16_t right_output = (uint16_t)pid_right.Out;
    if (left_output > PWM_MAX) left_output = PWM_MAX;
    if (right_output > PWM_MAX) right_output = PWM_MAX;
    
    // 应用PID输出到左右轮电机（前进：左轮CH2，右轮CH3）
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_output);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_output);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

// 后退 - 两个电机都反转，使用左右轮独立PID控制
void Motor_Backward(uint16_t target_speed)
{
	
    if (target_speed > PWM_MAX) target_speed = PWM_MAX;
    
    // 设置左右轮PID目标值
    pid_left.Target = (float)target_speed;
    pid_right.Target = (float)target_speed;
    
    // 更新左右轮PID控制器
    PID_Update(&pid_left);
    PID_Update(&pid_right);
    
    // 获取左右轮PID输出值并限制范围
    uint16_t left_output = (uint16_t)pid_left.Out;
    uint16_t right_output = (uint16_t)pid_right.Out;
    if (left_output > PWM_MAX) left_output = PWM_MAX;
    if (right_output > PWM_MAX) right_output = PWM_MAX;
    
    // 应用PID输出到左右轮电机（后退：左轮CH1，右轮CH4）
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_output);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_output);
}


// 左转 - 左电机反转，右电机正转
void Motor_TurnLeft(uint16_t speed_left, uint16_t speed_right)
{

    if (speed_left > PWM_MAX) speed_left= PWM_MAX;
    if (speed_right > PWM_MAX) speed_right= PWM_MAX;
	
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed_right);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed_left);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		
}

// 右转 - 左电机正转，右电机反转
void Motor_TurnRight(uint16_t speed_left, uint16_t speed_right)
{
	motor_is_stop = 0;
    if (speed_left > PWM_MAX) speed_left= PWM_MAX;
    if (speed_right > PWM_MAX) speed_right= PWM_MAX;
	
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed_left);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed_right);
}

// 连续PID控制更新 - 在主循环中调用


// 重置PID控制器（停止时调用）
void Motor_PID_Reset(void)
{
    pid_left.ErrorInt = 0;
    pid_right.ErrorInt = 0;
    pid_left.Error0 = 0;
    pid_right.Error0 = 0;
    pid_left.Error1 = 0;
    pid_right.Error1 = 0;
}


