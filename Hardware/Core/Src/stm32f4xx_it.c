/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t parse_buf[64];
uint16_t parse_len = 0;
uint8_t sync_mode = 0;         // 0: rest, 1: ongoing
uint8_t cur_data_len = 0;
char rdinfobuf[256];
int infolen;
volatile uint8_t uart3_busy = 0;
int fuckoff=0;

uint8_t txtundentifiedmsg[]="undentifiedmsg";
uint8_t txtnice[]="nice";
uint8_t txtwarn[]="warn";
uint8_t txterror[]="error code:";

extern uint8_t rx6buf[64];
extern uint8_t rx3buf[2];
extern uint8_t flag[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) uart3_busy = 0;
}

void Parseradar(uint8_t *data)
{
	uint8_t S  = data[0] & 0x01;
	if ((data[1] & 0x01) != 1) return;
	if (S==((data[0] >> 1) & 0x01)) return;
	
	infolen = sprintf(rdinfobuf,"{\"start\":%s,\"quality\":%d,\"angle\":%.2f,\"distance\":%.2f}\r\n",
	(S == 1) ? "true" : "false",
	(data[0] >> 2),
	(((((uint16_t)data[2]) << 7) | (data[1] >> 1))/64.0f),
	((((uint16_t)data[4] << 8) | data[3])/4.0f));
	
	if (infolen > 0) {
    HAL_UART_Transmit_IT(&huart3, (uint8_t*)rdinfobuf, infolen);
  }
}

void ParseData(void)
{
  uint16_t i = 0;

  while (i < parse_len)
  {
    if (parse_len - i >= 7 && parse_buf[i] == 0xA5 && parse_buf[i+1] == 0x5A)
    {
			switch(parse_buf[i+2])
			{
				case 0x05:
					cur_data_len = parse_buf[i+2];
					sync_mode = 1;
					break;
				case 0x03:
					cur_data_len = parse_buf[i+2];
					sync_mode = 1;
				break;
				default:
					HAL_UART_Transmit_IT(&huart3,txtundentifiedmsg,sizeof(txtundentifiedmsg)-1);
					
			}
      //HAL_UART_Transmit(&huart3, &parse_buf[i], 7, 1000);
			i += 7;
		}
		else if (sync_mode && (parse_len - i) >= cur_data_len)
		{
			if (cur_data_len == 3)
			{
				if (parse_buf[i]==0x00)
				{
					HAL_UART_Transmit(&huart3, txtnice, sizeof(txtnice)-1, 1000);
				}
				else if (parse_buf[i]==0x01)
				{
					HAL_UART_Transmit(&huart3, txtwarn, sizeof(txtwarn)-1, 1000);
				}
				else if (parse_buf[i]==0x02)
				{
					HAL_UART_Transmit(&huart3, txterror, sizeof(txterror)-1, 1000);
					parse_buf[i+1]+=0x30;
					HAL_UART_Transmit(&huart3, parse_buf+i+1, 1, 1000);
				}
				else
				{
					HAL_UART_Transmit(&huart3, flag, sizeof(flag), 1000);
				}
				sync_mode = 0;
				cur_data_len = 0;
			}
			else if(cur_data_len==5)
			{
				if(fuckoff==11)
				{
					Parseradar(&parse_buf[i]);
					fuckoff=0;
				}
				else fuckoff+=1;
			}	
			
			//HAL_UART_Transmit(&huart3, &parse_buf[i], cur_data_len, 1000);
			i += cur_data_len;
		}
		else if (!sync_mode)
      {
				HAL_UART_Transmit(&huart3, flag, sizeof(flag),100);
				HAL_UART_Transmit(&huart3, &parse_buf[i], 1, 1000);
				i += 1;
      }
       else
				{
           break;
				}
	}	
  // ?? parse_buf ??????
  if (i < parse_len)
  {
     memmove(parse_buf, &parse_buf[i], parse_len - i);
     parse_len = parse_len - i;
  }
  else
  {
    parse_len = 0;
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);

	  uint16_t remaining = __HAL_DMA_GET_COUNTER(huart6.hdmarx);
    uint16_t cur_len = sizeof(rx6buf) - remaining;

    static uint16_t last_pos = 0;
    uint16_t recv_len;

    if (cur_len >= last_pos)
    {
      recv_len = cur_len - last_pos;
    }
    else
    {
      recv_len = sizeof(rx6buf) - last_pos + cur_len;
    }
		
    if (cur_len != last_pos)
    {
      if (cur_len > last_pos)
      {
        //HAL_UART_Transmit_IT(&huart3, &rx6buf[last_pos], recv_len);
				memcpy(&parse_buf[parse_len], &rx6buf[last_pos], recv_len);
      }
      else
      {
				uint16_t first_part = sizeof(rx6buf) - last_pos;
        memcpy(&parse_buf[parse_len], &rx6buf[last_pos], first_part);
        memcpy(&parse_buf[parse_len + first_part], &rx6buf[0], cur_len);
        //HAL_UART_Transmit_IT(&huart3, &rx6buf[last_pos], sizeof(rx6buf) - last_pos);
        //HAL_UART_Transmit_IT(&huart3, &rx6buf[0], cur_len);
      }
			parse_len += recv_len;
    }
    last_pos = cur_len;
  }
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
