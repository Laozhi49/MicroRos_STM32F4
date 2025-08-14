/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "micro_ros.h"

#include "gpio.h"
#include "tim.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for microRosTask */
osThreadId_t microRosTaskHandle;
uint32_t microRosTaskBuffer[ 3000 ];
osStaticThreadDef_t microRosTaskControlBlock;
const osThreadAttr_t microRosTask_attributes = {
  .name = "microRosTask",
  .cb_mem = &microRosTaskControlBlock,
  .cb_size = sizeof(microRosTaskControlBlock),
  .stack_mem = &microRosTaskBuffer[0],
  .stack_size = sizeof(microRosTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Reboot_Check()
{
  static uint8_t RebootPin_status = GPIO_PIN_RESET;
  if((HAL_GPIO_ReadPin(Reboot_Pin_GPIO_Port,Reboot_Pin_Pin) == GPIO_PIN_SET) && (RebootPin_status == GPIO_PIN_RESET))
  {
    NVIC_SystemReset();
  }
    
  RebootPin_status = HAL_GPIO_ReadPin(Reboot_Pin_GPIO_Port,Reboot_Pin_Pin);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUltrasonicTask(void *argument);
void StartmicroRosTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UltrasonicTask */
  UltrasonicTaskHandle = osThreadNew(StartUltrasonicTask, NULL, &UltrasonicTask_attributes);

  /* creation of microRosTask */
  microRosTaskHandle = osThreadNew(StartmicroRosTask, NULL, &microRosTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUltrasonicTask */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void *argument)
{
  /* USER CODE BEGIN StartUltrasonicTask */
  /* Infinite loop */
  for(;;)
  {
    Reboot_Check();
    Ultrasonic_Start();
    osDelay(80);
  }
  /* USER CODE END StartUltrasonicTask */
}

/* USER CODE BEGIN Header_StartmicroRosTask */
/**
* @brief Function implementing the microRosTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmicroRosTask */
void StartmicroRosTask(void *argument)
{
  /* USER CODE BEGIN StartmicroRosTask */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10 ms

  // Initialize xLastWakeTime with the current tick count
  xLastWakeTime = xTaskGetTickCount();

  Micro_Ros_initial();
  /* Infinite loop */
  for(;;)
  {
    Micro_Ros_Process();
    //osDelay(5);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartmicroRosTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

