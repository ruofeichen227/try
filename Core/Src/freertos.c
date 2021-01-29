/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "beep.h"
#include "supervise.h"
#include "BMI088driver.h"
#include "AttitudeResolve.h"
#include "referee.h"
#include "fifo.h"
#include "robot.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
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
extern struct IMU_t bmi088;
extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ShootTask */
osThreadId_t ShootTaskHandle;
const osThreadAttr_t ShootTask_attributes = {
  .name = "ShootTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 128 * 4
};
/* Definitions for RobotCMDTask */
osThreadId_t RobotCMDTaskHandle;
const osThreadAttr_t RobotCMDTask_attributes = {
  .name = "RobotCMDTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
  .name = "RefereeTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for LED_Timer */
osTimerId_t LED_TimerHandle;
const osTimerAttr_t LED_Timer_attributes = {
  .name = "LED_Timer"
};
/* Definitions for Supervise_Timer */
osTimerId_t Supervise_TimerHandle;
const osTimerAttr_t Supervise_Timer_attributes = {
  .name = "Supervise_Timer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartImuTask(void *argument);
void StartShootTask(void *argument);
void StartChassisTask(void *argument);
void StartGimbalTask(void *argument);
void StartRobotCMDTask(void *argument);
void StartRefereeTask(void *argument);
void LedTimer(void *argument);
void SuperviseTimer(void *argument);

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

  /* Create the timer(s) */
  /* creation of LED_Timer */
  LED_TimerHandle = osTimerNew(LedTimer, osTimerPeriodic, NULL, &LED_Timer_attributes);

  /* creation of Supervise_Timer */
  Supervise_TimerHandle = osTimerNew(SuperviseTimer, osTimerPeriodic, NULL, &Supervise_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(LED_TimerHandle, 200);
	osTimerStart(Supervise_TimerHandle, 2);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of ShootTask */
  ShootTaskHandle = osThreadNew(StartShootTask, NULL, &ShootTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &GimbalTask_attributes);

  /* creation of RobotCMDTask */
  RobotCMDTaskHandle = osThreadNew(StartRobotCMDTask, NULL, &RobotCMDTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle = osThreadNew(StartRefereeTask, NULL, &RefereeTask_attributes);

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
		if(bmi088.accBiasFound)
		{
			Beep();
		}
    osDelay(30);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  while(BMI088_init()){}
  /* Infinite loop */
  for(;;)
  {
		imuDataHandle(&bmi088);				//BMI088数据解算
		LostCounterFeed(GYRO_INDEX);
		osDelay(1);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShootTask */
void StartShootTask(void *argument)
{
  /* USER CODE BEGIN StartShootTask */
  /* Infinite loop */
  for(;;)
  {
		ShootParamChange();
    osDelay(2);
  }
  /* USER CODE END StartShootTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
	  ChassisParamChange();
		osDelay(2);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void *argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  /* Infinite loop */
  for(;;)
  {
//		taskENTER_CRITICAL();
		GimbalParamChange();
//		taskEXIT_CRITICAL();
    osDelay(2);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartRobotCMDTask */
/**
* @brief Function implementing the RobotCMDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotCMDTask */
void StartRobotCMDTask(void *argument)
{
  /* USER CODE BEGIN StartRobotCMDTask */
	RobotParamInit();
  /* Infinite loop */
  for(;;)
  {
		RobotStateChange();
    osDelay(1);
  }
  /* USER CODE END StartRobotCMDTask */
}

/* USER CODE BEGIN Header_StartRefereeTask */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRefereeTask */
void StartRefereeTask(void *argument)
{
  /* USER CODE BEGIN StartRefereeTask */
	Referee_Data_Init();
	fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	USART6_Init();
  /* Infinite loop */
  for(;;)
  {
		Referee_Unpack_FIFO_Data();
    osDelay(3);
  }
  /* USER CODE END StartRefereeTask */
}

/* LedTimer function */
void LedTimer(void *argument)
{
  /* USER CODE BEGIN LedTimer */
	static int LED_cnt = 0;
	switch(LED_cnt)
	{
		case 0:
			HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
			break;
		case 1:
			HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
			break;
		case 2:
			HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
			break;
	}
	if(LED_cnt == 2)
		LED_cnt = 0;
	else
		LED_cnt ++;
  /* USER CODE END LedTimer */
}

/* SuperviseTimer function */
void SuperviseTimer(void *argument)
{
  /* USER CODE BEGIN SuperviseTimer */
	SuperviseTaskHandle();
  /* USER CODE END SuperviseTimer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
