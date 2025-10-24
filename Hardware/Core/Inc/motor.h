// motor.h

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"  // HAL 库定义
#include <stdint.h>         // int16_t

// 外部声明：CubeMX 在其他地方定义了 htim3，这里需要引用
extern TIM_HandleTypeDef htim3;
extern volatile int motor_is_stop;
// 电机驱动基础接口
void Motor_Init(void);
void Motor_SetSpeed(int16_t speedA, int16_t speedB);
void Motor_Stop(void);

// PID控制相关函数
void Motor_PID_Init(void);
void Motor_UpdateActualSpeed(float left_speed, float right_speed);
void Motor_PID_Reset(void);


// 电机运动控制接口
void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_TurnLeft(uint16_t speed_left, uint16_t speed_right);
void Motor_TurnRight(uint16_t speed_left, uint16_t speed_right);

#endif /* __MOTOR_H */
