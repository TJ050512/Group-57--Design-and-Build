/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU.h"
#include "syscalls.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "encoder_speed.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SensorData imuData;
uint32_t imu_prev_time = 0;
int speedon=0;
int mpuon=0;
uint8_t is_first_call = 1;  
float last_angle = 0.0f;
uint8_t rx3buf[2];
uint8_t rx6buf[64];
uint8_t flag[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
#define M_PI 3.14159265358979323846
Encoder_Data_t encoder_data;
uint32_t encoder_update_timer = 0;
uint32_t encoder_print_timer = 0;

uint8_t txtstop[]="stop";
uint8_t txtforward[]="forward";
uint8_t txtbackward[]="backward";
uint8_t txtleft[]="left";
uint8_t txtright[]="right";
uint8_t txtrdstop[]="radarstop";
uint8_t txtrdstart[]="radarstart";
uint8_t txtrdstate[]="radarstate: ";
uint8_t txtA5[]="A5 ";
char txtnewcommand[3];

uint8_t txtundentified[]="undentified";

// ????
float x_d = 0.0f;
float y_d = 0.0f;
float last_x = 0.0f;
float last_y= 0.0f;
float diff_x = 0.0f;
float diff_y = 0.0f;
 char encoder_info[128];
 double instant_distance=0.0;
// ????????????????
float prev_left_distance = 0.0f;
float prev_right_distance = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern void ParseData(void);
extern void Parseradar(uint8_t *data);	
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    //if (!bluetooth_connected) bluetooth_connected = 1;
		//HAL_UART_Transmit(&huart3, rx3buf,2,1000);
    switch (rx3buf[0])

		{
			case 0x00:
				switch(rx3buf[1])
				{
					case 0x00:
						HAL_UART_Transmit(&huart3,txtstop,sizeof(txtstop)-1,100);
						Motor_Stop();
						break;
					case 0x01:
						HAL_UART_Transmit(&huart3,txtforward,sizeof(txtforward)-1,100);
						Motor_Forward(150);
						break;
					case 0x02:
						HAL_UART_Transmit(&huart3,txtbackward,sizeof(txtbackward)-1,100);
						Motor_Backward(150);
						break;
					case 0x03:
						HAL_UART_Transmit(&huart3,txtleft,sizeof(txtleft)-1,100);
						Motor_TurnLeft(400,400);
						break;
					case 0x04:
						HAL_UART_Transmit(&huart3,txtright,sizeof(txtright)-1,100);
						Motor_TurnRight(400,400);
						break;
					case 0x05:
        // ???????
        Encoder_Data_Reset(&encoder_data);
        // ???????????
        x_d = 0.0f;
        y_d = 0.0f;
        prev_left_distance = 0.0f;
        prev_right_distance = 0.0f;
        HAL_UART_Transmit(&huart3, (uint8_t*)"Encoder Reset\r\n", 15, 1000);
					
					
        break;
					case 0x06:
						speedon=1-speedon;
					break;
					case 0x07:
						mpuon=1-mpuon;
					break;
					case 0x08: // ??:??????(????0x08 0x00???)
            if (rx3buf[1] == 0x00) {
               IMU_InitYawReference(&imuData); // ?????0°
               HAL_UART_Transmit(&huart3, (uint8_t*)"Angle reset to 0\n", 17, 100);
                }
          break;
					case 0x09: {
                     char msg[100];
                     float current_angle = imuData.yaw;  
                     sprintf(msg, "Current Angle: %.2f degrees\n", current_angle);
                     HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
                     float diff;
                     if (is_first_call) {
                         diff = current_angle - 0.0f;  // ???:???? - 0
											 
                         is_first_call = 0;            // ???????
											 
                     } else {
                         diff = current_angle - last_angle;  // ????:?? - ???
											   diff_x=x_d - last_x;
											 diff_y=y_d - last_y;
                     }
                     sprintf(msg, "Angle Difference (new - old): %.2f degrees\n X Difference:%.2f cm Y Difference:%.2f cm", diff,diff_x,diff_y);
                     HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
                     last_angle = current_angle;  // ???????????“???”
						         last_x = x_d;
										 last_y = y_d;
                     break;
                    }
					case 0x10:{
					motor_is_stop = 1;
						break;
					}
					case 0x11:{
					motor_is_stop = 0;
						break;
					}
			/*
			case 0xA525:
				HAL_UART_Transmit(&huart3, flag,10,1000);
				HAL_UART_Transmit(&huart6,rx3buf,2,1000);
				HAL_UART_Transmit(&huart3,rx3buf,2,1000);
				break;
			case 0xA520:
				HAL_UART_Transmit(&huart3, flag,10,1000);
				HAL_UART_Transmit(&huart6,rx3buf,2,1000);
				HAL_UART_Transmit(&huart3,rx3buf,2,1000);
				break;
			*/
		  default:
				HAL_UART_Transmit(&huart6,rx3buf,1,1000);
				HAL_UART_Transmit(&huart3,rx3buf,1,1000);
				}
				break;
			case 0xA5:
				switch(rx3buf[1])
				{
					case 0x25:
						HAL_UART_Transmit(&huart3,txtrdstop,sizeof(txtrdstop)-1,100);
						HAL_UART_Transmit(&huart6,rx3buf,2,100);
						break;
					case 0x20:
						HAL_UART_Transmit(&huart3,txtrdstart,sizeof(txtrdstart)-1,100);
						HAL_UART_Transmit(&huart6,rx3buf,2,100);
						break;
					case 0x52:
						if(motor_is_stop == 1){
						HAL_UART_Transmit(&huart3,txtrdstate,sizeof(txtrdstate)-1,100);
						HAL_UART_Transmit(&huart6,rx3buf,2,100);
						break;}
					default:
						HAL_UART_Transmit(&huart3,txtA5,sizeof(txtA5)-1,100);
						sprintf(txtnewcommand,"%02x",rx3buf[1]);
						HAL_UART_Transmit(&huart3,(uint8_t*)txtnewcommand,sizeof(txtnewcommand)-1,100);
						HAL_UART_Transmit(&huart6,rx3buf,2,100);
				}
				break;
		  default:
				HAL_UART_Transmit(&huart3,txtundentified,sizeof(txtundentified)-1,100);
		}
		HAL_UART_Receive_IT(&huart3, rx3buf, 2);
	}
}


/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    //if (!bluetooth_connected) bluetooth_connected = 1;
		//HAL_UART_Transmit(&huart3, rx3buf,2,1000);
		HAL_UART_Transmit(&huart6,flag,sizeof(flag),100);
      switch (rx3buf[0])
      {
      case 0x00:
        Motor_Stop();
        break;
      case 0x01:
        Motor_Forward(400);
        break;
      case 0x02:
        Motor_Backward(400);
        break;
      case 0x03:
        Motor_TurnLeft(400,400);
        break;
      case 0x04:
        Motor_TurnRight(400,400);
        break;
		  default:
				HAL_UART_Transmit(&huart6,rx3buf,2,100);
				HAL_UART_Transmit(&huart3,rx3buf,2,100);
    }
    HAL_UART_Receive_IT(&huart3, rx3buf, 2);
  }
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
float dt=0.0f;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	Motor_Init();        // ? ?????? PWM
	Encoder_Init();  // ??????
	Encoder_Data_Init();  // ??????????
	//HAL_UART_Receive_IT(&huart3, rx3buf, 1);
	IMU_Init();          // ? ?? MPU6500 ?????,? HAL_I2C_Init
	IMU_InitYawReference(&imuData);
  IMU_CalibrateGyroZ(&imuData);  
  imu_prev_time = HAL_GetTick();
	HAL_UART_Receive_IT(&huart3, rx3buf, sizeof(rx3buf));
	HAL_UART_Receive_DMA(&huart6, rx6buf, sizeof(rx6buf));
	
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	HAL_UART_Transmit(&huart3, (uint8_t*)"start trying\n", 14, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		ParseData();
		 // ???????(?10ms????)
    uint32_t current_time = HAL_GetTick();
    if (current_time - encoder_update_timer >= 10)
    {
        Encoder_Data_Update(&encoder_data);
        // ¸üÐÂPID¿ØÖÆÆ÷µÄÊµ¼ÊËÙ¶È·´À¡
        
        // ??:????
			if(motor_is_stop==0){
				Motor_UpdateActualSpeed(encoder_data.left_speed_cm_s, encoder_data.right_speed_cm_s);
        IMU_ReadSensor(&imuData);  // ?????????
        dt = (current_time - imu_prev_time) / 1000.0f;  // ??????(?)
        IMU_UpdateYaw(&imuData, dt);  // ??????
        imu_prev_time = current_time;  // ?????
			  
			  encoder_update_timer = current_time;
			        // ???????????
        double rad = imuData.yaw * M_PI / 180.0;
       
        // ??????????? - ?????
        float current_left_distance = encoder_data.left_distance_cm;
        float current_right_distance = encoder_data.right_distance_cm;
        float instant_left_distance = current_left_distance - prev_left_distance;
        float instant_right_distance = current_right_distance - prev_right_distance;
         instant_distance = (instant_left_distance + instant_right_distance) ; // ????
        
        // ????
        x_d = x_d + instant_distance * cos(rad);
        y_d = y_d + instant_distance * sin(rad);
        
			  
        // ?????????
        prev_left_distance = current_left_distance;
        prev_right_distance = current_right_distance;
    }
	}
    // ???????(?500ms????)
		if (current_time - encoder_print_timer >= 2000)
    {

        
        sprintf(encoder_info, "x:%.2f,y:%.2f",x_d, y_d);
			
        if(speedon==1)
				{
					HAL_UART_Transmit(&huart3, (uint8_t*)encoder_info, strlen(encoder_info), 1000);}
        encoder_print_timer = current_time;
				
			
    IMU_ReadSensor(&imuData);
		if(mpuon==1){
    IMU_PrintRawData(&imuData);
    IMU_PrintPhysicalValues(&imuData);
	}
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
