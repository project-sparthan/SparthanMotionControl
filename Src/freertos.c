/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "stm32f3xx_hal.h"
#include "usart.h"

#include "string.h"
#include <stdlib.h>
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
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim19;

uint16_t encoderCount[5];

/* Single byte to store input */
uint8_t byte;
uint16_t dutyCycle[5] = {0,0,0,0,0};
uint8_t direction = 0;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Debugginf functions for the UART */
void debugPrint(UART_HandleTypeDef *huart, char _out[]){ 
  HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
}
void debugPrintln(UART_HandleTypeDef *huart, char _out[]){
       HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
       char newline[2] = "\r\n";
       HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

/* Set callback called by the HAL_UART_IRQHandler */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* Receive one byte in interrupt mode */ 
    HAL_UART_Receive_IT(&huart1, &byte, 1);
    if (byte == 's'){
      for (int i = 0; i<5; i++) 
        dutyCycle[i] = 65000;
      direction = 1;
    }
    else if (byte == 'n'){
      for (int i = 0; i<5; i++) 
        dutyCycle[i] = 65000;   
      direction = 0;
      debugPrintln(&huart1, "OK");
    }
    else if (byte == 't'){
      for (int i = 0; i<5; i++) 
        dutyCycle[i] = 0;    
      debugPrintln(&huart1, "OK");
    }
  }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */

  /* Setup interrupt routine for USART communication */
  HAL_UART_Receive_IT(&huart1, &byte, 1);

  /* Initialize encoder and motors */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim19, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim19, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  for(;;)
  {
    
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, direction);
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, direction);
    HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, direction);
    HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, direction);
    HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, direction);

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, dutyCycle[0]);     //Motor 1
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, dutyCycle[1]);     //Motor 2
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, dutyCycle[2]);     //Motor 3
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, dutyCycle[3]);     //Motor 4
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, dutyCycle[4]);     //Motor 5
    encoderCount[0] = __HAL_TIM_GET_COUNTER(&htim2);              //Motor 1
    encoderCount[1] = __HAL_TIM_GET_COUNTER(&htim3);              //Motor 2
    encoderCount[2] = __HAL_TIM_GET_COUNTER(&htim4);              //Motor 3
    encoderCount[3] = __HAL_TIM_GET_COUNTER(&htim5);              //Motor 4
    encoderCount[4] = __HAL_TIM_GET_COUNTER(&htim19);             //Motor 5
    
    // Debug output
    debugPrint(&huart1, "Encoder Position: "); // print full line 
    char out [64];
    for (int i = 0; i < 5; i++){
      out [6*i] = '0' + (encoderCount[i]/10000)%10;
      out [6*i+1] = '0' + (encoderCount[i]/1000)%10;
      out [6*i+2] = '0' + (encoderCount[i]/100)%10;
      out [6*i+3] = '0' + (encoderCount[i]/10)%10;
      out [6*i+4] = '0' + encoderCount[i]%10;
      out [6*i+5] = ' ';
    }
    debugPrintln(&huart1, out); // print full line

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
