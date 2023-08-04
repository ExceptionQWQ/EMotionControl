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

#include "key.h"
#include "servo.h"
#include "usart.h"
#include "screen.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




enum MODE_STATE{
    MODE_STATE_NONE,
    MODE_STATE_CALI,
    MODE_STATE_RESET,
    MODE_STATE_BORDER,
    MODE_STATE_TRACK,
};

int mode_state = MODE_STATE_NONE;


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
/* Definitions for modeSelection */
osThreadId_t modeSelectionHandle;
const osThreadAttr_t modeSelection_attributes = {
  .name = "modeSelection",
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
void ModeSelection(void *argument);

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

  /* creation of modeSelection */
  modeSelectionHandle = osThreadNew(ModeSelection, NULL, &modeSelection_attributes);

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
  char msg[32];
  /* Infinite loop */
  for(;;)
  {
//      snprintf(msg, 32, "%d %d\r\n", mvRecvPoint.x, mvRecvPoint.y);
//      Debug(msg);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ModeSelection */

enum OPENMV_CMD
{
    OPENMV_CMD_STOP = 0x00,
    OPENMV_CMD_DETECT_RED_LASER = 0x01,
    OPENMV_CMD_DETECT_RECT = 0x02,
};

void OpenMVSendCMD(uint8_t cmd)
{
    HAL_UART_Transmit(&huart2, &cmd, 1, 100);
}


void DrawLine(struct Point2d start, struct Point2d end)
{
    int delay = 1;
    double step = 0.006;
    double len = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
    if (len < 1e-6) return ;
    for (double drawLen = 0; drawLen <= len; drawLen += step) {
        double x = start.x + (end.x - start.x) / len * drawLen;
        double y = start.y + (end.y - start.y) / len * drawLen;
        struct BallCoord ballCoord = CalcScreenServoCoord(x + screen_origin_pos.x, y + screen_origin_pos.y);
        struct ServoPos servoPos = CalcServoPos(ballCoord);
        servoXPos = servoPos.x; servoYPos = servoPos.y;
        UpdateScreen();
        osDelay(delay);
    }
    cursor_pos = end;
}

void DrawTo(struct Point2d to)
{
    DrawLine(cursor_pos, to);
}

void ModeBorder()
{
    double left = -0.2633, right = 0.262, top = 0.25, bottom = -0.25;
    struct Point2d aPt = {left, top};
    struct Point2d bPt = {right, top};
    struct Point2d cPt = {right, bottom};
    struct Point2d dPt = {left, bottom};

    DrawTo(aPt);
    DrawTo(bPt);
    DrawTo(cPt);
    DrawTo(dPt);
    DrawTo(aPt);
}


void ModeReset()
{
    struct Point2d origin = {0.0001, 0.0001};
    DrawTo(origin);
    UpdateScreen();
}

void ModeCali()
{
    OpenMVSendCMD(OPENMV_CMD_DETECT_RED_LASER);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

    volatile int tick = 0;
    //移动到原点
    DrawTo(mv_o_pos);
    osDelay(500);
    tick = mvUpdateTick;
    osDelay(500);
    while (tick == mvUpdateTick) {}
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    mv_o_pos_cali.x = mvRecvPoint.x;
    mv_o_pos_cali.y = mvRecvPoint.y;

    //移动到a点
    DrawTo(mv_a_pos);
    osDelay(500);
    tick = mvUpdateTick;
    osDelay(500);
    while (tick == mvUpdateTick) {}
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    mv_a_pos_cali.x = mvRecvPoint.x;
    mv_a_pos_cali.y = mvRecvPoint.y;

    //移动到b点
    DrawTo(mv_b_pos);
    osDelay(500);
    tick = mvUpdateTick;
    osDelay(500);
    while (tick == mvUpdateTick) {}
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    mv_b_pos_cali.x = mvRecvPoint.x;
    mv_b_pos_cali.y = mvRecvPoint.y;

    //移动到c点
    DrawTo(mv_c_pos);
    osDelay(500);
    tick = mvUpdateTick;
    osDelay(500);
    while (tick == mvUpdateTick) {}
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    mv_c_pos_cali.x = mvRecvPoint.x;
    mv_c_pos_cali.y = mvRecvPoint.y;

    //移动到d点
    DrawTo(mv_d_pos);
    osDelay(500);
    tick = mvUpdateTick;
    osDelay(500);
    while (tick == mvUpdateTick) {}
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    mv_d_pos_cali.x = mvRecvPoint.x;
    mv_d_pos_cali.y = mvRecvPoint.y;


    char msg[64] = {0};
    snprintf(msg, 64, "o %d %d\r\n", mv_o_pos_cali.x, mv_o_pos_cali.y);
    Debug(msg);
    snprintf(msg, 64, "a %d %d\r\n", mv_a_pos_cali.x, mv_a_pos_cali.y);
    Debug(msg);
    snprintf(msg, 64, "b %d %d\r\n", mv_b_pos_cali.x, mv_b_pos_cali.y);
    Debug(msg);
    snprintf(msg, 64, "c %d %d\r\n", mv_c_pos_cali.x, mv_c_pos_cali.y);
    Debug(msg);
    snprintf(msg, 64, "d %d %d\r\n", mv_d_pos_cali.x, mv_d_pos_cali.y);
    Debug(msg);


    //复位
    ModeReset();
    OpenMVSendCMD(OPENMV_CMD_STOP);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

}

void ModeTrack()
{
    OpenMVSendCMD(OPENMV_CMD_DETECT_RECT);
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

    osDelay(1000);

    volatile int tick = mvUpdateTick;
    osDelay(500);

    while (tick == mvUpdateTick) {}
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);


    struct Point2d aPt = CalcScreenPosFromOpenMVPos(mvRectA);
    struct Point2d bPt = CalcScreenPosFromOpenMVPos(mvRectB);
    struct Point2d cPt = CalcScreenPosFromOpenMVPos(mvRectC);
    struct Point2d dPt = CalcScreenPosFromOpenMVPos(mvRectD);
    DrawTo(aPt);
    DrawTo(bPt);
    DrawTo(cPt);
    DrawTo(dPt);
    DrawTo(aPt);

}

/**
* @brief Function implementing the modeSelection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModeSelection */
void ModeSelection(void *argument)
{
  /* USER CODE BEGIN ModeSelection */

    uint8_t fffff;
    HAL_UART_Receive_IT(&huart2, &fffff, 1);

    //校准
    screen_servo_origin_pos.y = 2283;
    screen_origin_pos = CalcScreenPosFromServoPos(screen_servo_origin_pos);
    ModeReset();
    osDelay(1000);

    ModeCali();

    ModeTrack();

  /* Infinite loop */
  for(;;)
  {
      ReadKeyBit();
      UpdateScreen();
//      ModeBorder();
      switch (mode_state) {
          case MODE_STATE_NONE:
              if (IsModeChange()) {
                  EchoModeSelectOk();
                  if (bit0 == 0 && bit1 == 0) {
                      mode_state = MODE_STATE_CALI;
                  } else if (bit0 == 1 && bit1 == 0) {
                      mode_state = MODE_STATE_RESET;
                  } else if (bit0 == 0 && bit1 == 1) {
                      mode_state = MODE_STATE_BORDER;
                  } else if (bit0 == 1 && bit1 == 1) {
                      mode_state = MODE_STATE_TRACK;
                  }
              }
              break;
          case MODE_STATE_CALI:
              ModeCali();
              mode_state = MODE_STATE_NONE;
              break;
          case MODE_STATE_RESET:
              ModeReset();
              mode_state = MODE_STATE_NONE;
              break;
          case MODE_STATE_BORDER:
              ModeBorder();
              mode_state = MODE_STATE_NONE;
              break;
          case MODE_STATE_TRACK:
              ModeTrack();
              mode_state = MODE_STATE_NONE;
              break;
      }
    osDelay(1);
  }
  /* USER CODE END ModeSelection */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

