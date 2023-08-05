/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "screen.h"
#include "servo.h"
#include "usart.h"s
#include "string.h"
#include "stdio.h"

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
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for trackingTask */
osThreadId_t trackingTaskHandle;
const osThreadAttr_t trackingTask_attributes = {
  .name = "trackingTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



volatile int mvUpdateTick = 0;
struct Point mvRecvPoint;
struct Point mvRectA;
struct Point mvRectB;
struct Point mvRectC;
struct Point mvRectD;

void Debug(char* msg)
{
    HAL_UART_Transmit(&huart1, msg, strlen(msg), 100);
}

void HandleOpenMVPackage(int cmd, uint8_t* data, int dataLen)
{
    switch (cmd) {
        case 1:
            mvUpdateTick += 1;
            mvRecvPoint.x = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
            mvRecvPoint.y = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
            break;
        case 2:
            mvUpdateTick += 1;
            mvRectA.x = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
            mvRectA.y = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
            mvRectB.x = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
            mvRectB.y = (int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7]);
            mvRectC.x = (int16_t)(((uint16_t)data[8] << 8) | (uint16_t)data[9]);
            mvRectC.y = (int16_t)(((uint16_t)data[10] << 8) | (uint16_t)data[11]);
            mvRectD.x = (int16_t)(((uint16_t)data[12] << 8) | (uint16_t)data[13]);
            mvRectD.y = (int16_t)(((uint16_t)data[14] << 8) | (uint16_t)data[15]);
            break;
        case 3:
            mvUpdateTick += 1;
            mvRectA.x = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
            mvRectA.y = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
            mvRectB.x = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
            mvRectB.y = (int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7]);
            break;
    }
}

//检查通用串口传输数据包
void CheckUartTransmitPackage(uint8_t* data, int* offset)
{
    if (*offset < 4) return ;
    if (data[0] != 0x12 || data[1] != 0x34) { //不是帧头，则左移一字节
        for (int i = 0; i < *offset - 1; ++i) {
            data[i] = data[i + 1];
        }
        *offset -= 1;
        return ;
    }
    //是帧头
    int dataLen = data[2];
    int cmd = data[3];
    uint8_t* messageData = data + 4;
    if (*offset >= dataLen + 4) {
        HandleOpenMVPackage(cmd, messageData, dataLen);
        *offset = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    static uint8_t uart2RecvBuff[1] = {};
    static uint8_t uart2Buff[128] = {};
    static int uart2Offset = 0;

    if (huart->Instance == USART2) { //openmv接口

        uart2Buff[uart2Offset++] = uart2RecvBuff[0];
        CheckUartTransmitPackage(uart2Buff, &uart2Offset);
        HAL_UART_Receive_IT(huart, uart2RecvBuff, 1);
    }
}


void EchoModeSelectOk()
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TrackingTask(void *argument);

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

  /* creation of trackingTask */
  trackingTaskHandle = osThreadNew(TrackingTask, NULL, &trackingTask_attributes);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TrackingTask */


enum OPENMV_CMD
{
    OPENMV_CMD_STOP = 0x00,
    OPENMV_CMD_DETECT_RED_LASER = 0x01,
    OPENMV_CMD_DETECT_RECT = 0x02,
    OPENMV_CMD_DETECT_TWO_LASER = 0x03,
};

void OpenMVSendCMD(uint8_t cmd)
{
    HAL_UART_Transmit(&huart2, &cmd, 1, 100);
}


struct PIDHandleDef
{
    double kp;
    double ki;
    double kd;
    double input;
    double target;
    double output;
    double error;
    double lastError;
    double lastError2;
};

double PIDTick(struct PIDHandleDef* pid)
{
    pid->lastError2 = pid->lastError;
    pid->lastError = pid->error;
    pid->error = pid->target - pid->input;

    pid->output = pid->kp * (pid->error - pid->lastError) +
                  pid->ki * (pid->error) +
                  pid->kd * (pid->error - 2 * pid->lastError + pid->lastError2);
    return pid->output;
}


/**
* @brief Function implementing the trackingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrackingTask */
void TrackingTask(void *argument)
{
  /* USER CODE BEGIN TrackingTask */
    osDelay(1000);

    uint8_t fffff;
    HAL_UART_Receive_IT(&huart2, &fffff, 1);

    OpenMVSendCMD(OPENMV_CMD_DETECT_TWO_LASER);

    UpdateScreen();

    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(1000);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);



    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(50);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(50);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(50);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(50);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(50);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

    osDelay(500);

    volatile int tick = mvUpdateTick;
    osDelay(100);

    int bxLast = SCREEN_SERVO_ORIGIN_X, byLast = SCREEN_SERVO_ORIGIN_Y;

    struct PIDHandleDef pidX = {};
    struct PIDHandleDef pidY = {};
    pidX.kp = 0.1; pidX.ki = 0.09; pidX.kd = 0.1;
    pidY.kp = 0.1; pidY.ki = 0.09; pidY.kd = 0.1;

  /* Infinite loop */
  for(;;)
  {
      if (tick != mvUpdateTick) {
          tick = mvUpdateTick;
          int ax = mvRectA.x, ay = mvRectA.y;
          if (ax == 0 && ay == 0) {
              osDelay(1);
              continue;
          }
          int bx = mvRectB.x, by = mvRectB.y;
          if (bx == 0 && by == 0) {
              bx = bxLast; by = byLast;
          }
          bxLast = bx; byLast = by;

          pidX.input = bx; pidX.target = ax;
          pidY.input = by; pidY.target = ay;
          servoXPos += PIDTick(&pidX);
          servoYPos += PIDTick(&pidY);

          UpdateScreen();

      }
    osDelay(1);
  }
  /* USER CODE END TrackingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

