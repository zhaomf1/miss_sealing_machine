/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "modbus_rtu.h"
#include "modbus_slave_reg.h"
#include "app_control.h"
#include "usart_comm.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
//任务句柄
osThreadId_t defaultTaskHandle;
osThreadId_t uartCommTaskHandle;        //与上位机通讯任务句柄
osThreadId_t tempCtrlTaskHandle;        //温控任务句柄
osThreadId_t compoundActionTaskHandle;  //复合动作任务句柄


//消息队列句柄
osMessageQueueId_t uartRxQueueHandle;   //串口接收消息队列

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t uartCommTask_attributes = {
    .name = "uartCommTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityRealtime,   //与上位机有时通信失败，提高优先级
};

const osThreadAttr_t tempCtrlTask_attributes = {
    .name = "tempCtrlTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t compoundAction_attributes = {
    .name = "compoundAction",
    .stack_size = 1024 * 3,   //曾为512，栈溢出致队列控制块被踩，改为3KB
    .priority = (osPriority_t)osPriorityNormal,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    modbus_create_rx_semaphore();     //初始化串口3接收信号量
    modbus_create_mutex();   //modbus初始化，初始化互斥锁

    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
     uartRxQueueHandle = osMessageQueueNew(4, sizeof(UartMsg_t), NULL);                     //帧数据随消息复制，深度4可避免缓冲覆盖并控制内存占用
    /* USER CODE END RTOS_QUEUES */

    /* 复合动作异步执行队列 (深度16，防止快速连续命令丢帧) */
    compound_action_queue = osMessageQueueNew(16, sizeof(CompoundActionMsg_t), NULL);

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);       //创建默认任务，喂狗
    uartCommTaskHandle = osThreadNew(uart_comm_task, NULL, &uartCommTask_attributes);       //创建与上位机通讯任务
    tempCtrlTaskHandle = osThreadNew(temp_ctrl_task, NULL, &tempCtrlTask_attributes);    //温度控制任务
    compoundActionTaskHandle = osThreadNew(compound_action_task, NULL, &compoundAction_attributes);/* 复合动作后台任务 */
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
    for (;;)
    {
        // 喂狗
        extern IWDG_HandleTypeDef hiwdg;
        HAL_IWDG_Refresh(&hiwdg);

        led_blink();

        osDelay(500);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



/* USER CODE END Application */
