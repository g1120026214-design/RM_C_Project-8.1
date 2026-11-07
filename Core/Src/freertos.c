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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "gm6020.h"
#include "pid.h"
#include "usart.h"
#include "string.h"
#include "tim.h"
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
PID_t speed_pid;
volatile float target_speed = 0.0f; //
volatile int16_t motor_voltage=0;
volatile uint8_t motor_online =0;
volatile uint8_t can_tx_ok = 0;
float Kp = 66.0f; //设置pid参数
float Ki = 60.0f;
float Kd = 0.013f;
float output_Max = 15000.0f;
float integrator_Max = 10000.0f;

static uint8_t rx_buffer[32];
//舵机相关
int angle = 0;               // 当前目标角度
int step = 20;               // 每次改变的角度
uint32_t last_key_tick = 0;
/* USER CODE END Variables */
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UartQueue */
osMessageQueueId_t UartQueueHandle;
const osMessageQueueAttr_t UartQueue_attributes = {
  .name = "UartQueue"
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};
/* Definitions for FeedbackQueue */
osMessageQueueId_t FeedbackQueueHandle;
const osMessageQueueAttr_t FeedbackQueue_attributes = {
  .name = "FeedbackQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void *argument);
void StartUartTask(void *argument);
void StartServoTask(void *argument);
void StartLedTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of UartQueue */
  UartQueueHandle = osMessageQueueNew (4, sizeof(UartMsg_t), &UartQueue_attributes);

  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (4, sizeof(float), &MotorQueue_attributes);

  /* creation of FeedbackQueue */
  FeedbackQueueHandle = osMessageQueueNew (4, sizeof(float), &FeedbackQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(StartServoTask, NULL, &ServoTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  PID_Init(&speed_pid, Kp, Ki, Kd,
      output_Max,integrator_Max);

  uint32_t wake_tick = osKernelGetTickCount();
  uint32_t last_tick = wake_tick;
  uint32_t  clock_tick = osKernelGetTickCount();
  uint32_t clock_last_tick = clock_tick;
  /* Infinite loop */
  for(;;)
  {
    wake_tick += 10;
    osDelayUntil(wake_tick); //固定任务周期

    uint32_t now = osKernelGetTickCount();
    uint32_t now2 = osKernelGetTickCount();
    float dt = (now - last_tick)
    / (float)osKernelGetTickFreq(); //两次记录tick的时间差值除以每秒的tick数 得到响应时间（单位s）
    last_tick = now;

    float new_speed;
    while (osMessageQueueGet(MotorQueueHandle,
      &new_speed,NULL,0)==osOK) {
      target_speed = new_speed;
    }

    motor_online=GM6020_IsOnline();
    if (motor_online && target_speed != 0.0f) {
      speed_pid.target = target_speed;
      speed_pid.feedback = motor_speed;
      motor_voltage = (int16_t)PID_Calc(&speed_pid,dt);
      if (now2 - clock_last_tick > 2000) {
      osMessageQueuePut(FeedbackQueueHandle,&speed_pid.feedback,0,0); //每2s放入反馈速度
        clock_last_tick = now2;
      }
    }else {
      PID_Reset(&speed_pid);
      motor_voltage = 0;
    }

    can_tx_ok = GM6020_SendVoltage(motor_voltage);


  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  UartMsg_t message={0};
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6,rx_buffer,
      sizeof(rx_buffer)) != HAL_OK) {
      Error_Handler();
      }
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);//关闭DMA过半中断
  /* Infinite loop */
  for(;;)
  { float speed_feedback;
    if (osMessageQueueGet(UartQueueHandle,&message
      ,NULL,10) == osOK) {
      float speed;
    if (strcmp(message.text,"motor+100")==0) {
      speed = 100.0f;
    }else if (strcmp(message.text,"motor0")==0) {
      speed = 0.0f;
    }else {
      continue;
    }

      osMessageQueuePut(MotorQueueHandle, &speed, 0, 0); //将收到的串口指令速度放入队列


    }
    if (osMessageQueueGet(FeedbackQueueHandle, &speed_feedback, NULL, 0) == osOK){
      char text[32];
      int length = snprintf(text, sizeof(text), "speed:%d rpm\r\n", //这一段接受数据一直乱码 ai帮忙优化成这样
                            (int)speed_feedback);
      HAL_UART_Transmit(&huart6, (uint8_t *)text, length, 100);
    }
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */
  /* Infinite loop */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                      500 + angle * 2000 / 180);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;) {
    motor_online=GM6020_IsOnline();
    if (motor_online!=0) { //电机在线就亮绿灯
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,0);
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    }
    if (motor_online == 0) { //离线的时候就灭灯亮呼吸灯
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      static int brightness=0;
      static int step=10;
      brightness+=step;
      if (brightness>=1000) step=-10;
      if (brightness<=0) step=10;
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, brightness);
    }osDelay(10);
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance != USART6) {
    return;
  }
  if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
    return;
  }
  UartMsg_t message={0};

  if (Size>0&&Size<sizeof(message.text)) {
    memcpy(message.text,rx_buffer,Size);
    message.text[Size]='\0';

    osMessageQueuePut(UartQueueHandle, &message, 0,0);
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
    rx_buffer,sizeof(rx_buffer)) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    uint32_t now = HAL_GetTick();
    if (now - last_key_tick > 200) {//按键消抖
      last_key_tick = now;
      angle += step;
      if (angle >= 180)
        step = -20;
      if (angle <= 0)
        step = 20;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
      500 + angle * 2000 / 180);
    }
  }
}
/* USER CODE END Application */

